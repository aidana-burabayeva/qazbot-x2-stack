"""Q&A sheet fast path: questions in the brief file that the robot answers verbatim, without the LLM.

The operator site writes them into the brief as
    - Вопрос: Где находится туалет?
      Также спрашивают: Где уборная?; Где WC?
      Ответ: Туалет — справа от входа.
A guest question that matches one (same words, small ASR slips allowed) gets the sheet's answer straight away:
no LLM round trip, and no chance of the model rewording a fact. An answer is written in one language, so it is
used only when the guest asks in that language; other languages go to the LLM, which sees the sheet in the brief.
"""
import difflib
import re

from .language import detect

_Q = re.compile(r"^\s*-\s*Вопрос:\s*(.+)$")
_ALSO = re.compile(r"^\s+Также спрашивают:\s*(.+)$")
_A = re.compile(r"^\s+Ответ:\s*(.+)$")
MATCH = 0.88          # whole-question similarity; a wrong fast answer is worse than a slow right one


def norm(text):
    return " ".join("".join(c for c in text.lower().replace("ё", "е") if c.isalnum() or c.isspace()).split())


def parse(brief):
    """[(variants, answer)] from the brief text."""
    out, q, also, ans = [], None, [], None

    def flush():
        if q and ans:
            out.append(([q] + also, ans))
    for line in (brief or "").splitlines():
        m = _Q.match(line)
        if m:
            flush()
            q, also, ans = m.group(1).strip(), [], None
            continue
        if q is None:
            continue
        m = _ALSO.match(line)
        if m:
            also = [v.strip() for v in m.group(1).split(";") if v.strip()]
            continue
        m = _A.match(line)
        if m:
            ans = m.group(1).strip()
    flush()
    return out


class FastAnswers(dict):
    """Drop-in for Turn.fast_answers: `.get(normalized_text, default)` also matches near-identical questions."""

    def __init__(self, brief=""):
        super().__init__()
        self.items_ = []
        for variants, answer in parse(brief):
            lang = detect(answer) or "ru"
            for v in variants:
                n = norm(v)
                if len(n) >= 4:
                    self.items_.append((n, {lang: answer}))
                    self.setdefault(n, {lang: answer})

    def get(self, key, default=None):
        if key in self:
            return self[key]
        best, hit = 0.0, None
        for n, ans in self.items_:
            r = difflib.SequenceMatcher(None, key, n).ratio()
            if r > best:
                best, hit = r, ans
        return hit if best >= MATCH else default
