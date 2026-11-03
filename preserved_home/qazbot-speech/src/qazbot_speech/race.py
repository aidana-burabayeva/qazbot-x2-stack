"""Cross-provider hedging for STT and TTS.

Both Soniox and Mangisoz stalled for 3-13 s on single calls in the 2026-10-15 measurements, at different
moments, so each is the other's hedge:
  * RaceTTS: the primary starts at once; if it has produced no audio after `hedge_sec`, the secondary starts
    too; the first to deliver audio wins and the other is cancelled.
  * DualSTT: the live Soniox stream is the main path; when the turn ends, Mangisoz gets the buffered audio at
    once. The first non-empty transcript wins (Mangisoz returned empty text for about half of loud-hall clips,
    so an empty answer never wins). The audio is buffered by its own task, so Soniox failing at once (no balance,
    closed socket) still leaves Mangisoz the whole turn: it is a real fallback, not only a hedge.
"""
import asyncio

from . import language


class RaceTTS:
    """TTS with hedging that keeps ONE voice as long as possible.

    `sources` are tried in order; source i starts early if the earlier ones have produced no audio after
    `schedule[i-1]` seconds, or at once if they fail. The first source to deliver audio wins; the others are
    cancelled. Put the same provider twice at the front (a second identical request, same voice) and a different
    provider last: its voice sounds different, and with foreign accents on ru/kk (Soniox's voices are English
    speakers), so it only joins late or after a failure.
    """

    def __init__(self, sources, secondary=None, hedge_sec=None, schedule=(0.9, 2.5)):
        """RaceTTS([a, a, b], schedule=(0.9, 2.5)) or the two-provider form RaceTTS(a, b, hedge_sec=0.9)."""
        if secondary is not None:
            sources = [sources, secondary]
            schedule = (hedge_sec if hedge_sec is not None else 0.9,)
        self.sources, self.schedule = list(sources), tuple(schedule)

    async def _pump(self, i, text, lang, out):
        try:
            async for chunk in self.sources[i].synth(text, lang):
                await out.put((i, "chunk", chunk))
            await out.put((i, "done", None))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            await out.put((i, "error", repr(e)))

    async def synth(self, text, lang):
        out, tasks, winner, errors = asyncio.Queue(), {}, None, 0
        loop = asyncio.get_event_loop()
        t0 = loop.time()

        def start(i):
            tasks[i] = asyncio.ensure_future(self._pump(i, text, lang, out))

        start(0)
        try:
            while True:
                nxt = len(tasks)
                wait = None
                if winner is None and nxt < len(self.sources):
                    wait = max(0.0, t0 + self.schedule[min(nxt - 1, len(self.schedule) - 1)] - loop.time())
                try:
                    i, kind, val = await asyncio.wait_for(out.get(), wait)
                except asyncio.TimeoutError:
                    start(len(tasks))
                    continue
                if winner is not None and i != winner:
                    continue
                if kind == "chunk":
                    if winner is None:
                        winner = i
                        for j, t in tasks.items():
                            if j != i:
                                t.cancel()
                    yield val
                elif kind == "done":
                    return
                else:
                    errors += 1
                    if winner == i:
                        raise RuntimeError("TTS failed mid-audio: %s" % val)
                    if len(tasks) < len(self.sources):
                        start(len(tasks))
                    elif errors >= len(tasks):
                        raise RuntimeError("all TTS providers failed: %s" % val)
        finally:
            for t in tasks.values():
                t.cancel()


class DualSTT:
    def __init__(self, live, batch, extra_wait=0.0):
        self.live, self.batch = live, batch      # live: transcribe_chunks/prewarm; batch: transcribe(pcm)
        self.extra_wait = extra_wait

    async def prewarm(self):
        await self.live.prewarm()

    async def transcribe_chunks(self, chunks, on_text=None, on_end=None):
        # One pump reads the microphone chunks. The live stream gets them from a queue, so a live provider that
        # fails, closes early or stops reading (Soniox with no balance closes at once) cannot stop the audio from
        # being buffered, and the batch provider still starts when the turn ends.
        audio, batch_task, feed = [], {}, asyncio.Queue()

        async def pump():
            try:
                async for c in chunks:
                    audio.append(c)
                    feed.put_nowait(c)
            finally:
                feed.put_nowait(None)
            # the turn has ended: Mangisoz starts on the buffered audio while Soniox finalizes
            batch_task["t"] = asyncio.ensure_future(self.batch.transcribe(b"".join(audio)))

        async def tee():
            while True:
                c = await feed.get()
                if c is None:
                    return
                yield c

        pump_task = asyncio.ensure_future(pump())
        live_task = asyncio.ensure_future(self.live.transcribe_chunks(tee(), on_text=on_text, on_end=on_end))
        try:
            pending, best = {live_task, pump_task}, None
            while pending:
                done, pending = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
                for d in done:
                    if d.exception():
                        continue
                    if d is pump_task:
                        if "t" in batch_task:
                            pending.add(batch_task["t"])
                        continue
                    text, lang = d.result()
                    if text.strip():
                        return text.strip(), (lang or language.detect(text))
                    best = best or (text, lang)
            return best or ("", None)
        finally:
            for t in [live_task, pump_task] + list(batch_task.values()):
                if not t.done():
                    t.cancel()
                elif not t.cancelled():
                    t.exception()   # mark retrieved: a failed loser must not log 'never retrieved'
