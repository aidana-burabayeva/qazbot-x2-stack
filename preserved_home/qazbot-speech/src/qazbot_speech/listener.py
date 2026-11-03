"""Listen for one utterance from a mic and stream it to STT while the guest is still speaking.

`mic` is an async iterator of (t_seconds, pcm_bytes) frames of about 100 ms. The VAD and the Endpointer
decide where the turn ends; the STT sees the audio live, so its result is ready right after the end of
speech. A short pre-roll is kept so the first syllable is not clipped.
"""
import asyncio
import collections
import time
from dataclasses import dataclass

from .endpoint import Endpointer
from .vad import EnergyVad, dbfs


@dataclass
class Heard:
    text: str
    lang: object
    end_t: object         # mic time of the last speech frame seen by our endpointer (None if STT ended first)
    decided_t: object     # mic time our endpointer closed the turn (None if Soniox's own endpoint came first)
    end_wall: float = 0.0       # time.monotonic() at the last speech frame: speech-end -> result is measured from it
    by_stt: bool = False        # True when the STT's semantic endpoint ended the turn before ours
    peak: float = 0.0           # loudest frame of the utterance, dBFS: background talk is much quieter than a guest at the mic


async def listen_once(mic, stt, vad=None, endpointer=None, preroll_frames=3, level_cb=None, log=None, max_wait=60.0, gate=None, on_release=None):
    """`log(msg)` is called at speech start, turn end and STT result. `max_wait` bounds the STT after the
    mic loop ends; a failure inside the mic loop is raised here, never swallowed (it used to hang forever)."""
    vad, ep = vad or EnergyVad(), endpointer or Endpointer()
    pre = collections.deque(maxlen=preroll_frames)
    queue, state = asyncio.Queue(), {"decided": None, "end": None, "wall": 0.0, "peak": -96.0}

    async def pump():
        """Read the mic until the turn ends; push frames of the utterance to the STT stream."""
        started = False
        try:
            async for t, pcm in mic:
                if state.get("stop"):
                    break
                level = dbfs(pcm)
                if level_cb:
                    level_cb(level)
                # The gate sees EVERY frame (it learns how loud the robot's own echo is from the quiet ones too).
                allowed = gate.allows(level) if (gate and not started) else True
                speech = vad.is_speech(level) and allowed    # not the robot's own voice or its tail
                if speech:
                    state["wall"] = time.monotonic()
                if not started:
                    if speech:
                        started = True
                        if log:
                            log("speech start at %.1fs, level %.1f dBFS, gate %.1f" % (t, level, vad.threshold))
                        for f in pre:
                            await queue.put(f)
                        await queue.put(pcm)
                        state["peak"] = level
                        state["t_start"] = time.monotonic()
                        ep.feed(t, True)
                    else:
                        pre.append(pcm)
                    continue
                await queue.put(pcm)
                state["peak"] = max(state["peak"], level)
                if on_release:
                    # 0.45 s of silence: the answer can start now. Not when the turn is likely to go on
                    # (trailing "и", ",") or the transcript is empty; once per pause.
                    if speech:
                        state["released"] = False
                    elif (ep.release_ready and not state.get("released") and ep.partial.strip()
                          and ep.required_wait() < ep.max_wait):
                        state["released"] = True
                        on_release(ep.partial, state["peak"])
                rule, tail = ep.required_wait(), ep.partial[-40:]   # before feed(): a finished turn resets the endpointer
                u = ep.feed(t, speech)
                if u:
                    state["decided"], state["end"] = t, u.end
                    if log:
                        log("turn ended at %.1fs (silence rule %.2fs, partial %r)" % (t, rule, tail))
                    break
        except BaseException as e:
            state["error"] = e
            raise
        finally:
            await queue.put(None)      # the STT stream must always be closed, or it waits for audio forever

    async def frames():
        while True:
            f = await queue.get()
            if f is None:
                return
            yield f

    pump_task = asyncio.ensure_future(pump())
    stt_task = asyncio.ensure_future(
        stt.transcribe_chunks(frames(), on_text=ep.set_partial, on_end=(lambda: log and log("Soniox end hint"))))
    try:
        # `max_wait` bounds the turn from the moment speech starts, NOT the idle wait for someone to speak: a quiet
        # room used to raise a TimeoutError (and restart the mic) every 60 s.
        while True:
            done, _ = await asyncio.wait({stt_task}, timeout=1.0)
            if done:
                break
            if state.get("t_start") is not None and time.monotonic() - state["t_start"] > max_wait:
                raise asyncio.TimeoutError("STT did not finish %.0f s after speech started" % max_wait)
        text, lang = stt_task.result()
    finally:
        if not stt_task.done():
            stt_task.cancel()
        # Never cancel the pump while it waits on the mic: cancelling an await inside the shared mic generator
        # closes it for good, and every later listen would end at once with an empty result. Ask it to stop at
        # its next frame instead (about 100 ms), and cancel only if it does not.
        state["stop"] = True
        try:
            await asyncio.wait_for(asyncio.shield(pump_task), 1.5)
        except (asyncio.TimeoutError, Exception):
            pump_task.cancel()
    if "error" in state:
        raise state["error"]
    if log:
        log("STT result: lang=%s text=%r" % (lang, text[:80]))
    return Heard(text, lang, state["end"], state["decided"], state["wall"], state["decided"] is None, state["peak"])
