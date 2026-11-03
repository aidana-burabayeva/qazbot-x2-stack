"""What the TTS voice is handed to read out, per language.

Adapted in part from earlier QazBot robot code (see NOTICE.md). Venue-specific names are left out: they
belong in a venue config, not here.

The model writes product names in Latin letters whatever language it answers in; each Mangisoz voice reads
Latin differently (measured on the robot 2026-09-15: ru read QazBot as "Газбот", AgiBot as "Агибот", X2 as
"Экстарой"). So names are respelled for the voice about to read them, after the model answered and before the
text is synthesised; the conversation history is not touched.
"""
import re

_SEP = r"[\s\-‐-―_]*"


def _name(first, second):
    return re.compile(r"\b" + first + _SEP + second + r"\b", re.IGNORECASE)


_RESPELLINGS = {
    "ru": ((_name("qaz", "bot"), "Казбот"), (_name("agi", "bot"), "Аджибот"),
           (re.compile(r"\b[XХ]2\b"), "Икс Два")),
    "kk": ((_name("qaz", "bot"), "Қазбот"), (_name("agi", "bot"), "Аджибот"),
           (re.compile(r"\b[XХ]2\b"), "Икс Два")),
    "en": ((_name("qaz", "bot"), "Kazbot"),),
    # Stress, Russian only: a combining acute moves it off the end of "роботов" (heard back as "работав").
    # Not "+", which is read out as "плюс"; not in Kazakh, where the mark broke the voice.
    "ru_stress": ((re.compile(r"\b([Рр])о(?!\u0301)ботов\b"), "\1о\u0301ботов"),),
}

# Typography the model reaches for and a voice has no sound for (it broke Kazakh answers, 2026-09-18).
_TYPOGRAPHY = (
    (re.compile("[‐‑‒]"), "-"),
    (re.compile(r"\s*[–—―]\s*"), ", "),
    (re.compile("[«»“”„‘’‚‛]"), ""),
    (re.compile("[   ]"), " "),
    (re.compile("…"), "."),
)


def speakable(text, tts_lang):
    for pattern, spoken in _TYPOGRAPHY:
        text = pattern.sub(spoken, text)
    for pattern, spoken in _RESPELLINGS.get(tts_lang, ()):
        text = pattern.sub(spoken, text)
    for pattern, spoken in _RESPELLINGS.get(tts_lang + "_stress", ()):
        text = pattern.sub(spoken, text)
    return text


def to_synthesise(text, tts_lang):
    """Exactly what goes to the TTS stream: names respelled and a space after the last word. The space is
    not cosmetic: Mangisoz sometimes ends the audio mid-sound, and in Russian a trailing space cut the clipped
    last syllable from 2 of 5 runs to 0 of 5 (robot, 2026-09-15)."""
    text = speakable(text or "", tts_lang).rstrip()
    return text + " " if text else ""
