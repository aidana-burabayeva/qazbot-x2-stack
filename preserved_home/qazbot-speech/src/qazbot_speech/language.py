"""Reply language. Only kk, ru and en exist; there is no other language contract."""
import re

LANGS = ("kk", "ru", "en")
_KK_ONLY = set("әіңғүұқөһӘІҢҒҮҰҚӨҺ")
_CYR = re.compile(r"[а-яёәіңғүұқөһ]", re.I)
_LAT = re.compile(r"[a-z]", re.I)


def detect(text):
    """Language used most in `text`: en if Latin letters dominate; else kk if Kazakh-only letters appear,
    else ru. Returns None for text with no letters."""
    cyr = len(_CYR.findall(text))
    lat = len(_LAT.findall(text))
    if not cyr and not lat:
        return None
    if lat > cyr:
        return "en"
    kk = sum(1 for ch in text if ch in _KK_ONLY)
    return "kk" if kk >= 2 or (kk >= 1 and kk / max(cyr, 1) > 0.04) else "ru"


def reply_language(stt_lang, text, previous):
    """Which language to answer in.

    The STT's language label flips on short phrases ("John." -> en, a one-word Russian line -> another), so a
    short utterance never changes the conversation's language. Kazakh letters in the text are decisive; the
    script of the text overrules a label that contradicts it; a bare "kk" label on plain Russian words only
    counts inside a Kazakh conversation.
    """
    script = detect(text)
    words = len(re.findall(r"\w+", text))
    if script == "kk":
        return "kk"
    if previous and words <= 2:
        # One or two words: the label is unreliable ("John." -> en). A Cyrillic "ru" label is still believed
        # (Kazakh would show Kazakh letters or a kk label); anything else keeps the conversation's language.
        return "ru" if (script == "ru" and stt_lang == "ru") else previous
    if stt_lang == "kk" and script == "ru":
        return "kk" if previous == "kk" else "ru"
    if stt_lang in LANGS and (script is None or script == stt_lang or (script == "ru" and stt_lang == "kk")):
        return stt_lang
    return script or previous or "ru"


_TURKISH_ONLY = set("ığşİĞŞ")


def looks_foreign(text):
    """A transcript that is not kk/ru/en speech: mostly letters outside Latin and Cyrillic (a noise or a
    background voice read as Arabic, Chinese...), or Latin text with letters English never uses. Dropping it is
    safer than answering it; the robot speaks only kk, ru and en."""
    letters = [c for c in text if c.isalpha()]
    if not letters:
        return False
    ok = sum(1 for c in letters if re.match(r"[a-zA-Zа-яА-ЯёЁәіңғүұқөһӘІҢҒҮҰҚӨҺ]", c))
    if ok / len(letters) < 0.7:
        return True
    return any(c in _TURKISH_ONLY for c in letters)


def voice_lang(sentence, reply_lang):
    """The language a sentence is WRITTEN in, which picks the voice. The reply language comes from the guest's
    question; the model sometimes answers in another one, and the wrong voice reading it sounds like a foreign
    accent. Kazakh letters decide kk; Latin decides en; Cyrillic without Kazakh letters is ru when it is a real
    sentence (4+ words), else it keeps the reply language."""
    script = detect(sentence)
    if script in ("kk", "en"):
        return script
    if script == "ru":
        return "ru" if len(re.findall(r"\w+", sentence)) >= 4 else (reply_lang or "ru")
    return reply_lang or "ru"
