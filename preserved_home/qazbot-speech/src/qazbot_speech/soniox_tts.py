"""Soniox streaming TTS over REST. Yields raw 16-bit mono PCM chunks as they arrive.

Measured on Nagatoro 2026-10-15: usual first chunk 0.35-0.8 s, but about 1 call in 4 stalled for 3-10 s
(total up to 23 s), warm-up or not. So a second identical request is hedged after `hedge_sec` without a
first chunk; whichever delivers first wins and the other is cancelled.
"""
import asyncio

import aiohttp

URL = "https://tts-rt.soniox.com/tts"


class SonioxTTS:
    def __init__(self, api_key, voices=None, default_voice="Adrian", sample_rate=24000, hedge_sec=1.0):
        self.key = api_key
        self.voices = voices or {}       # per-language voice override
        self.default_voice, self.sample_rate, self.hedge_sec = default_voice, sample_rate, hedge_sec
        self._session = None

    def _http(self):
        if self._session is None or self._session.closed:
            self._session = aiohttp.ClientSession(headers={"Authorization": "Bearer " + self.key})
        return self._session

    async def _one(self, body, out, tag):
        try:
            async with self._http().post(URL, json=body, timeout=aiohttp.ClientTimeout(total=30)) as r:
                if r.status != 200:
                    await out.put((tag, "error", "HTTP %d" % r.status))
                    return
                async for chunk in r.content.iter_chunked(4096):
                    if chunk:
                        await out.put((tag, "chunk", chunk))
            await out.put((tag, "done", None))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            await out.put((tag, "error", repr(e)))

    async def synth(self, text, lang, copies=2):
        body = {"model": "tts-rt-v2", "language": lang, "voice": self.voices.get(lang, self.default_voice),
                "audio_format": "pcm_s16le", "sample_rate": self.sample_rate, "text": text}
        out, tasks, winner, started = asyncio.Queue(), [], None, 0

        def start():
            nonlocal started
            tasks.append(asyncio.ensure_future(self._one(body, out, started)))
            started += 1

        start()
        failed = 0
        try:
            while True:
                wait = self.hedge_sec if (winner is None and started < copies) else None
                try:
                    tag, kind, val = await asyncio.wait_for(out.get(), wait)
                except asyncio.TimeoutError:
                    start()
                    continue
                if winner is not None and tag != winner:
                    continue
                if kind == "chunk":
                    if winner is None:
                        winner = tag
                        for i, t in enumerate(tasks):
                            if i != tag:
                                t.cancel()
                    yield val
                elif kind == "done":
                    return
                else:
                    failed += 1
                    if winner == tag:
                        raise RuntimeError("Soniox TTS failed mid-audio: %s" % val)
                    if started < copies:
                        start()
                    elif failed >= started:
                        raise RuntimeError("Soniox TTS failed: %s" % val)
        finally:
            for t in tasks:
                t.cancel()

    async def warm(self, langs=("kk", "ru", "en")):
        """Open the connection and prime each language with a one-word synth; audio is discarded."""
        for lang in langs:
            try:
                async for _ in self.synth(".", lang, copies=1):
                    break
            except Exception:
                pass
