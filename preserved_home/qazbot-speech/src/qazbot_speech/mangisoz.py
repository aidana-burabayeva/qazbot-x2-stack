"""Mangisoz (ISSAI) providers. Request shapes follow the Digital Bridge client: `X-API-Key` header,
multipart `/stt/transcribe`, JSON `/expressive-tts/stream` returning raw 16-bit mono PCM (rate in the
`X-Sample-Rate` header, 24000 in every measurement).

Measured 2026-10-15 on the laptop: TTS first chunk 0.30-0.41 s (one English call stalled 13 s); STT is
batch, about 0.65 s for a 6 s clip, so it can only start once the turn has ended.
Voices that Digital Bridge used: kk ali, ru maxim, en harry.
"""
import io
import wave

import aiohttp

from .audio_tail import PauseCutter, with_sacrificial_pause
from .speech_text import to_synthesise

VOICES = {"kk": "ali", "ru": "maxim", "en": "harry"}
PAUSE_LANGS = ("kk",)   # measured: the pause helps Kazakh and makes ru/en run on (audio_tail.py)


def pcm_to_wav(pcm, rate):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)
    return buf.getvalue()


class _Http:
    def __init__(self, api_key, base_url):
        self.key, self.base = api_key, base_url.rstrip("/")
        self._session = None

    def http(self):
        if self._session is None or self._session.closed:
            self._session = aiohttp.ClientSession(headers={"X-API-Key": self.key})
        return self._session


class MangisozSTT(_Http):
    def __init__(self, api_key, base_url, sample_rate=16000, timeout=15):
        super().__init__(api_key, base_url)
        self.rate, self.timeout = sample_rate, timeout

    async def transcribe(self, pcm):
        """Return (text, None). The API does not report the language; the Turn detects it from the text."""
        form = aiohttp.FormData()
        for k in ("lang", "language"):
            form.add_field(k, "auto")
        form.add_field("response_format", "json")
        form.add_field("audio", pcm_to_wav(pcm, self.rate), filename="utterance.wav", content_type="audio/wav")
        async with self.http().post(self.base + "/stt/transcribe", data=form,
                                    timeout=aiohttp.ClientTimeout(total=self.timeout)) as r:
            if r.status != 200:
                raise RuntimeError("Mangisoz STT HTTP %d" % r.status)
            data = await r.json(content_type=None)
        return str(data.get("text") or "").strip(), None


class MangisozTTS(_Http):
    sample_rate = 24000

    def __init__(self, api_key, base_url, voices=None):
        super().__init__(api_key, base_url)
        self.voices = dict(VOICES, **(voices or {}))

    async def synth(self, text, lang):
        """Kazakh gets the sacrificial pause (the stream otherwise ends mid-word about 1 time in 5); the pause
        is cut out before it reaches the speaker. ru and en get a trailing space instead (see speech_text)."""
        text = to_synthesise(text, lang)
        cutter = None
        if lang in PAUSE_LANGS:
            text = with_sacrificial_pause(text)
        body = {"text": text, "voice": self.voices.get(lang, "maxim"), "language": lang, "emotion": "neutral"}
        async with self.http().post(self.base + "/expressive-tts/stream", json=body,
                                    timeout=aiohttp.ClientTimeout(total=30)) as r:
            if r.status != 200:
                raise RuntimeError("Mangisoz TTS HTTP %d" % r.status)
            rate = int(r.headers.get("X-Sample-Rate", self.sample_rate))
            cutter = PauseCutter(rate) if lang in PAUSE_LANGS else None
            async for chunk in r.content.iter_chunked(4096):
                if not chunk:
                    continue
                if cutter is None:
                    yield chunk
                    continue
                out, stop = cutter.feed(chunk)
                if out:
                    yield out
                if stop:
                    break
            if cutter is not None:
                tail = cutter.finish()
                if tail:
                    yield tail
