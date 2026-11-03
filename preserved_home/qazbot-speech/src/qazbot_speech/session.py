"""Wake/sleep and conversation memory.

Default (no wake name): the robot wakes on speech and sleeps after `idle_sleep` s without speech.
With wake names set: it wakes only when an utterance contains one of the names.
Memory is kept `memory_keep` s after sleep, then cleared so the next guest does not inherit it.
Time is injected, so the whole thing is testable without sleeping.
"""
import re

# Latin <-> Cyrillic letters so "Azamat", "Азамат" and "азамат" all match, however the STT wrote the name.
_LAT = [("sh", "ш"), ("ch", "ч"), ("zh", "ж"), ("kh", "х"), ("ya", "я"), ("yu", "ю"), ("yo", "ё"),
        ("ts", "ц"), ("a", "а"), ("b", "б"), ("c", "к"), ("d", "д"), ("e", "е"), ("f", "ф"), ("g", "г"),
        ("h", "х"), ("i", "и"), ("j", "ж"), ("k", "к"), ("l", "л"), ("m", "м"), ("n", "н"), ("o", "о"),
        ("p", "п"), ("q", "қ"), ("r", "р"), ("s", "с"), ("t", "т"), ("u", "у"), ("v", "в"), ("w", "в"),
        ("x", "кс"), ("y", "й"), ("z", "з")]
_KK = str.maketrans({"ә": "а", "ө": "о", "ү": "у", "ұ": "у", "қ": "к", "ғ": "г", "ң": "н", "һ": "х", "і": "и",
                     "ё": "е", "й": "и", "ы": "и", "э": "е", "ь": "", "ъ": ""})


def fold(text):
    """Map a name to one comparable Cyrillic skeleton."""
    t = text.lower()
    for lat, cyr in _LAT:
        t = t.replace(lat, cyr)
    t = t.translate(_KK)
    return re.sub(r"[^а-я]", "", re.sub(r"(.)\1+", r"\1", t))


def mentions_name(text, names):
    words = [fold(w) for w in re.findall(r"[\w'’-]+", text)]
    return any(fold(n) and fold(n) in words for n in names)


class Session:
    def __init__(self, clock, idle_sleep=15.0, memory_keep=120.0, names=()):
        self.clock, self.idle_sleep, self.memory_keep = clock, idle_sleep, memory_keep
        self.names = tuple(names)
        self.awake = False
        self.last_activity = None
        self.slept_at = None
        self.history = []

    def tick(self):
        """Call regularly (or before each decision): applies sleep and memory expiry."""
        now = self.clock()
        if self.awake and now - self.last_activity >= self.idle_sleep:
            self.awake, self.slept_at = False, self.last_activity + self.idle_sleep
        if not self.awake and self.slept_at is not None and now - self.slept_at >= self.memory_keep:
            self.history.clear()
            self.slept_at = None

    def hear(self, text):
        """An utterance was transcribed. Returns True if the robot should answer it."""
        self.tick()
        if self.awake:
            self.last_activity = self.clock()
            return True
        if self.names and not mentions_name(text, self.names):
            return False
        self.awake, self.last_activity, self.slept_at = True, self.clock(), None
        return True

    def spoke(self):
        """The robot finished speaking: the idle timer restarts from now."""
        if self.awake:
            self.last_activity = self.clock()
