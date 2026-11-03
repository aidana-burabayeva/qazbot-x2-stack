"""Hearing the guest while the robot talks, without hearing the robot itself.

Two layers, because neither alone is enough with a hand mic that has no loudspeaker reference channel:

1. EchoGate (level): while the robot speaks, the mic's own level is the robot's echo. The gate learns how loud
   that echo is (90th percentile of recent levels) and only lets a frame count as the guest when it is
   `margin_db` above the echo AND above the room gate, for `sustain_sec` in a row: that is a barge-in. For
   `tail_grace` after the robot stops, the same rule applies (Digital Bridge: 0.1 s grace made a robot answer
   itself, 0.6 s fixed it). A hand mic held at the mouth is far louder than the echo, so a real interruption
   passes; the robot's own voice does not.
2. own_echo (text): a transcript that is mostly what the robot said a moment ago is the robot hearing itself,
   whatever the levels said, and is dropped (Digital Bridge: transcript_guard.strip_own_echo).

This is a gate, not true acoustic echo cancellation (AEC needs the playback reference signal and a delay
estimate). With the Fifine at the guest's mouth it is enough; the robot's internal array hears the hall and
the speaker equally, so barge-in there is effectively off by design.
"""
import collections
import re
import time


class EchoGate:
    def __init__(self, vad, is_speaking, clock=time.monotonic, margin_db=8.0, sustain_sec=0.3,
                 tail_grace=0.6, frame_sec=0.1, on_barge_in=None):
        self.vad, self.is_speaking, self.clock = vad, is_speaking, clock
        self.margin, self.sustain, self.tail_grace, self.frame = margin_db, sustain_sec, tail_grace, frame_sec
        self.on_barge_in = on_barge_in
        self.echo = collections.deque(maxlen=50)    # mic levels seen while the robot was speaking
        self.mute_until = 0.0
        self._was_speaking = False
        self._loud = 0.0
        self.barging = False

    def echo_estimate(self):
        if len(self.echo) < 5:
            return self.vad.threshold + 10.0         # unknown echo: be conservative
        s = sorted(self.echo)
        return s[int(0.9 * (len(s) - 1))]

    def barge_threshold(self):
        return max(self.vad.threshold, self.echo_estimate() + self.margin)

    def allows(self, level):
        """May a frame at `level` dBFS count as the guest speaking? Call once per frame."""
        now, speaking = self.clock(), self.is_speaking()
        if self._was_speaking and not speaking and not self.barging:
            self.mute_until = now + self.tail_grace   # the robot just stopped: its tail is still in the room
        self._was_speaking = speaking
        if speaking or now < self.mute_until:
            if speaking:
                self.echo.append(level)
            self._loud = self._loud + self.frame if level >= self.barge_threshold() else 0.0
            if self._loud >= self.sustain:
                if not self.barging:
                    self.barging = True
                    if self.on_barge_in:
                        self.on_barge_in()
                    self._was_speaking, self.mute_until = False, 0.0   # the guest has the floor now
                return True
            return False
        self.barging, self._loud = False, 0.0
        return True


_WORD = re.compile(r"\w+", re.UNICODE)


def _words(text):
    return [w for w in _WORD.findall(text.lower())]


def own_echo(heard, said, now, window=20.0, threshold=0.6):
    """True when `heard` is the robot's own recent speech picked up by its mic.

    `said` is an iterable of (time, sentence) for what the robot spoke. Compares word sets: the share of the
    heard words that appear in what the robot said within `window` seconds.
    """
    hw = _words(heard)
    if not hw:
        return False
    recent = set()
    for t, sentence in said:
        if now - t <= window:
            recent.update(_words(sentence))
    if not recent:
        return False
    return sum(1 for w in hw if w in recent) / len(hw) >= threshold
