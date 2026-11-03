"""Cut a streamed reply into sentences so the first sentence can go to TTS before the LLM finishes."""
import re

_END = re.compile(r"(?<=[.!?…])\s+")
_CLAUSE = re.compile(r"(?<=[,;:—–])\s+")


class SentenceStream:
    def __init__(self, min_chars=6, first_clause_chars=22):
        """The FIRST chunk may end at a clause break (comma, colon, dash) once it has `first_clause_chars`
        characters: speech starts a sentence-length earlier. Later chunks wait for full sentences."""
        self.buf, self.min_chars, self.first_clause_chars = "", min_chars, first_clause_chars
        self.emitted = 0

    def feed(self, piece):
        self.buf += piece
        out = []
        while True:
            if not self.emitted and not out:
                c = _CLAUSE.search(self.buf)
                e = _END.search(self.buf)
                if c and c.start() + 1 >= self.first_clause_chars and (not e or c.start() < e.start()):
                    out.append(self.buf[:c.start() + 1].strip())
                    self.buf = self.buf[c.end():]
                    self.emitted += 1
                    continue
            m = _END.search(self.buf)
            if not m or m.start() + 1 < self.min_chars:
                break
            out.append(self.buf[:m.start() + 1].strip())
            self.buf = self.buf[m.end():]
            self.emitted += 1
        return out

    def flush(self):
        rest, self.buf = self.buf.strip(), ""
        self.emitted += 1 if rest else 0
        return [rest] if rest else []
