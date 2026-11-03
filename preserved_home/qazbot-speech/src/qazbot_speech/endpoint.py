"""Adaptive end-of-turn detection.

Fed one VAD decision per audio frame plus the latest partial transcript. The guest has finished when the
silence since the last speech reaches a wait that depends on what was just said:
  * sentence-final punctuation (. ? !)            -> short wait (min_wait, default 0.6 s); one or two
    words only (a bare greeting) -> short_wait, default 1.0 s
  * trailing connective or filler ("и", "ну", ...) -> long wait (max_wait, default 1.2 s)
  * anything else                                  -> in between
`release_ready` turns true after `release` seconds of silence (default 0.45 s): the caller may start STT
finalization speculatively then. If speech resumes before the turn ends, nothing is cut: the utterance
simply continues (segment gluing).
Bounds can be overridden per deployment (a loud hall wants longer waits).
"""
import re
from dataclasses import dataclass
from typing import Optional

_FILLER = re.compile(
    r"(?:^|\s)(и|а|но|ну|да|что|как|или|если|потому|который|значит|ээ|эм|"
    r"және|мен|бірақ|яғни|сонда|егер|және де|"
    r"and|but|so|well|um|uh|or|because|if|the|a)[\s,]*$", re.I)
_FINAL = re.compile(r"[.?!…]\s*$")


@dataclass
class Utterance:
    start: float
    end: float          # time of the last speech frame


class Endpointer:
    def __init__(self, release=0.45, min_wait=0.6, max_wait=1.2, max_turn=15.0, short_wait=1.0):
        self.release, self.min_wait, self.max_wait, self.max_turn = release, min_wait, max_wait, max_turn
        self.short_wait = max(short_wait, min_wait)   # waits for turns of one or two words
        self.reset()

    def reset(self):
        self.start: Optional[float] = None
        self.last_speech: Optional[float] = None
        self.partial = ""
        self.now = 0.0

    def set_partial(self, text):
        self.partial = text or ""

    def required_wait(self):
        text = self.partial.strip()
        if not text:
            return self.max_wait
        if _FINAL.search(text):
            # "Привет." alone is usually the start of a sentence, not the end of the turn.
            bare = len(text.split()) <= 2 and not text.endswith("?")   # a question is a complete turn; a bare "Привет." is not
            return self.short_wait if bare else self.min_wait
        if _FILLER.search(text) or text.endswith(","):
            return self.max_wait
        return (self.min_wait + self.max_wait) / 2

    @property
    def release_ready(self):
        return self.last_speech is not None and self.now - self.last_speech >= self.release

    def feed(self, t, is_speech):
        """Return an Utterance when the turn ended at time `t`, else None."""
        self.now = t
        if is_speech:
            if self.start is None:
                self.start = t
            self.last_speech = t
            if t - self.start >= self.max_turn:  # hard cap, like the hall recipe's turn limit
                return self._end()
            return None
        if self.last_speech is None:
            return None
        if t - self.last_speech >= self.required_wait():
            return self._end()
        return None

    def _end(self):
        u = Utterance(self.start, self.last_speech)
        self.reset()
        return u
