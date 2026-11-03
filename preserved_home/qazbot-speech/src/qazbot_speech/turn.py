"""The Turn: one guest utterance in, events out. This is the single test seam of the service."""
import asyncio
import time
import collections
from dataclasses import dataclass, field

from . import gestures, language
from .sentences import SentenceStream


@dataclass
class Event:
    kind: str          # transcript | gesture | sentence | audio | bridge | fast_path
    data: object = None
    t: float = 0.0     # seconds since the turn started


class Prefetch:
    """The LLM answer started speculatively while the guest's silence was still short (0.45 s). If the final
    transcript matches, `Turn.run` takes this stream and the first sentence is ready sooner; else it is dropped."""

    def __init__(self, llm, brief, history, text, lang):
        self.text, self.lang, self.hist_len = text, lang, len(history)
        self.q = asyncio.Queue()
        self.task = asyncio.ensure_future(self._fill(llm, brief, list(history)))

    async def _fill(self, llm, brief, history):
        try:
            async for piece in llm.stream(brief, history, self.text, self.lang):
                self.q.put_nowait(piece)
            self.q.put_nowait(None)
        except asyncio.CancelledError:
            raise
        except Exception as err:
            self.q.put_nowait(err)

    async def stream(self):
        while True:
            item = await self.q.get()
            if item is None:
                return
            if isinstance(item, Exception):
                raise item
            yield item

    def cancel(self):
        self.task.cancel()


@dataclass
class Turn:
    stt: object
    llm: object
    tts: object
    speaker: object
    brief: str = ""
    fast_answers: dict = field(default_factory=dict)    # normalized question -> {lang: answer}
    bridge_phrases: dict = field(default_factory=dict)  # lang -> "one moment" text
    history: list = field(default_factory=list)
    last_lang: str = "ru"
    clock: object = time.monotonic
    said: object = field(default_factory=lambda: collections.deque(maxlen=8))   # (time, sentence) the robot spoke

    tools: list = field(default_factory=list)           # context tools: .matches(text), async .context(text, brief)
    _prefetch: object = None

    def prefetch(self, partial):
        """Start the LLM on the partial transcript (the guest paused 0.45 s). Replaces an earlier prefetch."""
        text = (partial or "").strip()
        self.cancel_prefetch()
        if len(text) < 4 or any(t.matches(text) for t in self.tools):
            return          # a tool must fetch its facts first: the early answer would lack them
        lang = language.reply_language(None, text, self.last_lang)
        self._prefetch = Prefetch(self.llm, self.brief, self.history, text, lang)

    def cancel_prefetch(self):
        if self._prefetch is not None:
            self._prefetch.cancel()
            self._prefetch = None

    def _take_prefetch(self, text, lang):
        pf, self._prefetch = self._prefetch, None
        if pf is None:
            return None
        if _norm(pf.text) == _norm(text) and pf.lang == lang and pf.hist_len == len(self.history):
            return pf
        pf.cancel()
        return None

    async def run(self, audio=None, heard=None):
        """`heard` = (text, stt_lang) when a listener already streamed the audio to STT."""
        t0 = self.clock()
        events = []

        def emit(kind, data=None):
            events.append(Event(kind, data, self.clock() - t0))

        text, stt_lang = heard if heard is not None else await self.stt.transcribe(audio)
        lang = language.reply_language(stt_lang, text, self.last_lang)
        self.last_lang = lang
        emit("transcript", {"text": text, "lang": lang})

        motion = gestures.greeting_motion(text)
        if motion:  # fired before speech starts and never awaited by the reply
            await self.speaker.gesture(motion)
            emit("gesture", motion)

        fast = self.fast_answers.get(_norm(text), {}).get(lang)
        if fast:
            self.cancel_prefetch()
            emit("fast_path", fast)
            await self._speak(fast, lang, emit)
            self._remember(text, fast)
            return events

        spoken, stream = [], SentenceStream()
        try:
            head, silent, decided = "", False, False
            pf = self._take_prefetch(text, lang)
            if pf:
                emit("prefetch_hit")
            brief = self.brief
            for tool in self.tools:
                if tool.matches(text):
                    extra = await tool.context(text, self.brief)
                    if extra:
                        brief = brief + "\n\n" + extra
                        emit("tool", extra[:80])
            source = pf.stream() if pf else self.llm.stream(brief, self.history, text, lang)
            async for piece in source:
                if not decided:                            # the model may answer <silent>: look before speaking
                    head += piece
                    if "<silent" in head.lower():
                        silent = True
                        break
                    if len(head) < 12 and not any(c in head for c in ".!?"):
                        continue                           # too little text to know yet
                    decided, piece = True, head            # a real answer: release everything held back
                for s in stream.feed(piece):
                    spoken.append(s)
                    await self._speak(s, lang, emit)
            if silent:
                emit("silent", text)
                return events
            if not decided and head:                      # the whole answer was shorter than the look-ahead
                for s in stream.feed(head):
                    spoken.append(s)
                    await self._speak(s, lang, emit)
            for s in stream.flush():
                if s.strip().lower().startswith("<silent"):
                    emit("silent", text)
                    return events
                spoken.append(s)
                await self._speak(s, lang, emit)
        except Exception:
            if not spoken:
                emit("bridge", lang)
                await self._speak(self.bridge_phrases.get(lang, ""), lang, emit)
        if spoken:
            self._remember(text, " ".join(spoken))
        return events

    async def _speak(self, sentence, lang, emit):
        if not sentence:
            return
        if self.said and _norm(self.said[-1][1]) == _norm(sentence) and self.clock() - self.said[-1][0] < 8.0:
            emit("repeat_skipped", sentence)    # the model (or a retry) produced the same sentence twice in a row
            return
        emit("sentence", sentence)
        self.said.append((self.clock(), sentence))
        voice = language.voice_lang(sentence, lang)     # the voice follows the language the sentence is written in
        first = True
        async for chunk in self.tts.synth(sentence, voice):
            if first:
                emit("audio", len(chunk))
                first = False
            await self.speaker.play(chunk)
        if hasattr(self.speaker, "flush"):
            await self.speaker.flush()     # a short sentence may never have filled the speaker's priming buffer

    def _remember(self, user, reply):
        self.history += [{"role": "user", "content": user}, {"role": "assistant", "content": reply}]
        del self.history[:-16]


def _norm(text):
    return " ".join("".join(c for c in text.lower() if c.isalnum() or c.isspace()).split())
