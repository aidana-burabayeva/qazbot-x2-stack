"""The service loop: listen for an utterance, gate it with the session, run the Turn, record telemetry."""
import asyncio
import time

from .echo import EchoGate, own_echo
from .language import looks_foreign
from .listener import Heard, listen_once
from .vad import EnergyVad


class Service:
    def __init__(self, mic_factory, stt, turn, session, telemetry, clock=time.monotonic, persistent_mic=False, vad=None, log=None):
        """persistent_mic=True: open the mic once and keep reading it between turns (a real USB mic must not be
        reopened per utterance: audio would be lost in between). False: a fresh mic per step (replay/tests)."""
        self.mic_factory, self.stt, self.turn = mic_factory, stt, turn
        self.session, self.telemetry, self.clock = session, telemetry, clock
        self.persistent_mic, self._mic_it, self.log = persistent_mic, None, log
        self._turn_task, self.gate, self._pending = None, None, ""
        self._last_text = ("", 0.0)
        self._mute_until, self.tail_grace = 0.0, 0.6      # 0.1 s grace made a robot answer itself; 0.6 s fixed it
        self.vad = vad or EnergyVad()   # one gate for the whole run: calibration must survive between turns

    def _mic(self):
        if not self.persistent_mic:
            return self.mic_factory()
        if self._mic_it is None:
            self._mic_it = _Shared(self.mic_factory().__aiter__())
        return self._mic_it

    async def close(self):
        if self._mic_it is not None:
            try:
                await self._mic_it.it.aclose()
            except RuntimeError:       # "already running": the listener is mid-read; drop it, a new mic is built next
                pass
            self._mic_it = None

    async def _flush_mic(self, mute_until):
        """Drop audio that piled up in the capture buffer while the robot was speaking (and the robot's own
        voice with it). Backlog frames come back at once; real-time frames take about 100 ms each, so the
        first slow frame after the mute window means we have caught up."""
        if self._mic_it is None:
            return
        it, give_up = self._mic_it.it, self.clock() + 3.0
        while self.clock() < give_up:
            t0 = self.clock()
            try:
                await asyncio.wait_for(it.__anext__(), 1.0)
            except (asyncio.TimeoutError, StopAsyncIteration):
                return
            if self.clock() - t0 > 0.05 and self.clock() >= mute_until:
                return

    # ---- full duplex: keep listening while the robot speaks ----------------------------------------------

    @property
    def _speaker(self):
        return self.turn.speaker

    def _barge_in(self):
        """The guest talked over the robot: stop it at once."""
        if self.log:
            self.log("barge-in: guest over the robot's voice, stopping it")
        task = self._turn_task
        self._pending = ""
        if task is not None and not task.done():
            task.cancel()
        if hasattr(self._speaker, "interrupt"):
            self._speaker.interrupt()

    async def run(self):
        """Listen forever. A turn runs in the background; the mic keeps being read, so the guest can
        interrupt, and the robot's own voice is gated out (EchoGate + own_echo)."""
        self.gate = EchoGate(self.vad, lambda: bool(getattr(self._speaker, "is_speaking", False)),
                             clock=self.clock, tail_grace=self.tail_grace, on_barge_in=self._barge_in)
        while True:
            await self.stt.prewarm()
            heard = await listen_once(self._mic(), self.stt, vad=self.vad, level_cb=self._level,
                                      log=self.log, gate=self.gate, on_release=self._on_release)
            text = heard.text.strip()
            if not text:
                self.turn.cancel_prefetch()
                continue
            if looks_foreign(text):
                if self.log:
                    self.log("ignored a transcript that is not kk/ru/en speech: %r" % text[:40])
                self.turn.cancel_prefetch()
                continue
            need = self.vad.awake_peak if self.session.awake else self.vad.wake_peak
            if heard.peak < need:
                if self.log:
                    self.log("ignored background talk (peak %.1f dBFS < %.1f): %r" % (heard.peak, need, text[:50]))
                self.turn.cancel_prefetch()
                continue
            if own_echo(text, self.turn.said, self.clock()):
                if self.log:
                    self.log("ignored own voice heard back: %r" % text[:60])
                self.turn.cancel_prefetch()
                continue
            if not self.session.hear(text):
                self.turn.cancel_prefetch()
                continue
            norm = " ".join(text.lower().split())
            if norm == self._last_text[0] and self.clock() - self._last_text[1] < 6.0:
                if self.log:
                    self.log("ignored the same transcript twice in a row: %r" % text[:60])
                self.turn.cancel_prefetch()
                continue
            self._last_text = (norm, self.clock())
            busy = self._turn_task is not None and not self._turn_task.done()
            if busy and not getattr(self._speaker, "is_speaking", False) and self._pending:
                # The robot has not started speaking: this is the guest finishing their thought in two pieces
                # ("Маған көмектесші." ... "Әрине, болады."). Merge, so neither piece goes unanswered.
                text = self._pending.strip() + " " + text
                heard = Heard(text, heard.lang, heard.end_t, heard.decided_t, heard.end_wall, heard.by_stt, heard.peak)
                if self.log:
                    self.log("merged with the pending question: %r" % text[:80])
            if busy:
                self._turn_task.cancel()               # replaced by the merged question, or by a new topic
            self._pending = text
            self._turn_task = asyncio.ensure_future(self._respond(heard))

    def _on_release(self, partial, peak):
        """The guest paused 0.45 s: start the LLM on what was said so far (cancelled if the words change)."""
        need = self.vad.awake_peak if self.session.awake else self.vad.wake_peak
        if peak >= need and not looks_foreign(partial):
            self.turn.history = self.session.history
            self.turn.prefetch(partial)

    async def _respond(self, heard):
        try:
            self.turn.history = self.session.history
            events = await self.turn.run(heard=(heard.text, heard.lang))
            self.telemetry.record_turn(events)
            self.telemetry.set_health("turn", True)     # clears an earlier failure flag
            if self.log:
                if any(e.kind == "silent" for e in events):
                    self.log("stayed silent (not addressed to me): %r" % heard.text[:60])
                else:
                    self.log("reply [%s]: %s" % (heard.lang, " | ".join(str(e.data) for e in events if e.kind in ("sentence", "repeat_skipped"))))
            if hasattr(self._speaker, "drain"):
                await self._speaker.drain()
            self.session.spoke()
            self._pending = ""
        except asyncio.CancelledError:
            if hasattr(self._speaker, "interrupt"):
                self._speaker.interrupt()
            raise

    async def step(self):
        """One utterance. Returns the telemetry record of the turn, or None if the robot stayed quiet."""
        if self.persistent_mic:
            await self._flush_mic(self._mute_until)
        await self.stt.prewarm()   # socket open before the guest finishes: connect costs about 2.5 s
        heard = await listen_once(self._mic(), self.stt, vad=self.vad, level_cb=self._level, log=self.log)
        if not heard.text.strip() or not self.session.hear(heard.text):
            return None
        self.turn.history = self.session.history       # one memory, owned by the session
        events = await self.turn.run(heard=(heard.text, heard.lang))
        self.session.spoke()
        self._mute_until = self.clock() + self.tail_grace      # the robot's own tail must not be heard as a turn
        return self.telemetry.record_turn(events)

    def _level(self, dbfs_level):
        self.telemetry.mic["level_dbfs"] = round(dbfs_level, 1)

    def clear_memory(self):
        self.session.history.clear()


class _Shared:
    """Lets `async for` pick up the same underlying mic iterator again on the next turn."""
    def __init__(self, it):
        self.it = it

    def __aiter__(self):
        return self.it
