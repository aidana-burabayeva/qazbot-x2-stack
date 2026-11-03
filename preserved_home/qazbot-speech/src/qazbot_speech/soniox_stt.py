"""Soniox realtime STT (stt-rt-v5) for one utterance of PCM audio.

`transcribe` streams the utterance and returns as soon as Soniox marks the endpoint. Language id is on, so
the reply language follows what the guest actually spoke. Live streaming while the guest is still talking
comes later, with the mic layer; this class already accepts audio in chunks via `transcribe_chunks`.
"""
import asyncio
import json
import time

import websockets

URL = "wss://stt-rt.soniox.com/transcribe-websocket"
OURS = ("kk", "ru", "en")


class SonioxSTT:
    def __init__(self, api_key, sample_rate=16000, endpoint_sensitivity=0.0, timeout=15):
        self.key, self.rate = api_key, sample_rate
        self.sensitivity, self.timeout = endpoint_sensitivity, timeout

    async def prewarm(self):
        """Open the socket before the guest finishes speaking (call when the wake gate opens).
        Measured 2026-10-15 on the laptop: connect + TLS alone is about 2.5 s, so never connect per turn."""
        if getattr(self, "_warm", None) is None:
            self._warm = await websockets.connect(URL, extra_headers={"Authorization": "Bearer " + self.key})

    def _config(self):
        return {"model": "stt-rt-v5", "audio_format": "pcm_s16le", "sample_rate": self.rate, "num_channels": 1,
                "language_hints": list(OURS), "enable_language_identification": True,
                "enable_endpoint_detection": True, "endpoint_sensitivity": self.sensitivity}
        # The <end> marker is only a HINT now (it makes the final punctuation arrive sooner). It must never end
        # the turn: on 2026-10-15 it cut "Привет, кто президент Казахстана?" after "Привет.".

    async def transcribe(self, audio):
        """Replay one recorded utterance at real-time pace, then silence until Soniox marks the endpoint.
        (Soniox processes audio at real-time speed; sending it all at once only adds delay.)"""
        step = self.rate * 2 // 10  # 100 ms of s16 mono

        async def chunks():
            t0, n = time.monotonic(), 0  # absolute deadlines: per-chunk sleeps drift on Windows timers
            for i in range(0, len(audio), step):
                yield audio[i:i + step]
                n += 1
                await asyncio.sleep(max(0.0, t0 + n * 0.1 - time.monotonic()))
            for _ in range(20):  # up to 2 s of silence; stops early when the endpoint arrives
                yield bytes(step)
                n += 1
                await asyncio.sleep(max(0.0, t0 + n * 0.1 - time.monotonic()))
        return await self.transcribe_chunks(chunks())

    async def transcribe_chunks(self, chunks, on_text=None, on_end=None):
        """`on_text(partial)` is called with the transcript so far (final plus not-yet-final tokens)."""
        text, langs = "", {}
        it = chunks.__aiter__()
        # Connect only when the first audio exists. Connecting first and then waiting for speech left the socket
        # idle for as long as the room was quiet, and Soniox drops idle streams (seen 2026-10-15: the guest
        # spoke after about 20 s and the stream was already dead). Audio queues up while we connect.
        try:
            first = await it.__anext__()
        except StopAsyncIteration:
            return "", None
        ws = getattr(self, "_warm", None)
        self._warm = None
        if ws is None:
            ws = await websockets.connect(URL, extra_headers={"Authorization": "Bearer " + self.key})
        try:
            await ws.send(json.dumps(self._config()))
            done = asyncio.Event()

            async def reader():
                nonlocal text
                async for msg in ws:
                    d = json.loads(msg)
                    if "error_code" in d:
                        raise RuntimeError("Soniox STT %s: %s" % (d["error_code"], d.get("error_message")))
                    pending = ""
                    for t in d.get("tokens", []):
                        if t["text"] == "<end>":
                            if on_end:
                                on_end()          # a hint to the caller; the stream goes on
                            continue
                        if not t.get("is_final"):
                            pending += t["text"]
                        if t.get("is_final"):
                            text += t["text"]
                            lg = t.get("language")
                            if lg:
                                langs[lg] = langs.get(lg, 0) + len(t["text"])
                    if on_text and d.get("tokens"):
                        on_text(text + pending)
                    if d.get("finished"):
                        done.set()
                        return

            rt = asyncio.ensure_future(reader())
            try:
                await ws.send(first)
                async for c in it:
                    await ws.send(c)
                    if done.is_set():
                        break
                await ws.send("")  # no more audio: Soniox finalizes
                await asyncio.wait_for(done.wait(), self.timeout)
            except asyncio.TimeoutError:
                if rt.done() and rt.exception():  # surface the reader's real error, not a bare timeout
                    raise rt.exception()
                raise
            finally:
                rt.cancel()
        finally:
            asyncio.ensure_future(ws.close())  # closing handshake takes seconds; never block the turn on it
        lang = max(langs, key=langs.get) if langs else None
        return text.strip(), (lang if lang in OURS else None)
