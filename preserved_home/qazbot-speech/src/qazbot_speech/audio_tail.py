# Adapted from earlier QazBot robot code; see NOTICE.md.
"""Did this synthesis arrive complete, or did the API cut the last word off?

Mangisoz's /expressive-tts/stream ends the stream early, at random. Measured
2026-09-21 with the same sentence eight times: the audio came back between
4.95 s and 6.83 s long, and a local transcription of the short ones lost the
final syllable ("... жауап бер" for "... жауап беремін").

A complete clip ends in silence - the speaker stops, and a few hundred
milliseconds of room tone follow. A truncated one ends mid-vowel, at full
amplitude. That is the whole test here, and it is cheap and local: no second
API call, no transcription.

    complete    trailing quiet 276 ms, last 50 ms peak 24
    truncated   trailing quiet   0 ms, last 50 ms peak 22124

Callers retry on a false: tools/render_demo_audio.py re-renders a prepared
clip until it comes back clean, and the TTS node re-requests a live answer
once before speaking it.

Pure: no ROS, no network, no numpy. test/test_audio_tail.py covers it.
"""
import struct

#: A sample this quiet is silence for our purposes. S16 full scale is 32767;
#: real "silence" from this API measured 1-35, and room tone in a rendered
#: clip stays under 300.
SILENCE_LEVEL = 300

#: A complete clip has at least this much quiet at the end. The gap is wide:
#: truncated runs measured 0 ms, complete ones 91 ms to 464 ms. 60 ms sits in
#: the gap, and a clip at 91 ms was confirmed complete by transcribing it.
MIN_TRAILING_QUIET_MS = 60


def samples(pcm):
    """S16LE bytes as a tuple of ints. An odd trailing byte is ignored."""
    usable = len(pcm) - (len(pcm) % 2)
    return struct.unpack("<%dh" % (usable // 2), bytes(pcm[:usable]))


def trailing_quiet_ms(pcm, rate, silence_level=SILENCE_LEVEL):
    """How many milliseconds of quiet sit at the end of this audio."""
    if rate <= 0:
        return 0
    quiet = 0
    for value in reversed(samples(pcm)):
        if abs(value) > silence_level:
            break
        quiet += 1
    return int(quiet * 1000 / rate)


def ends_cleanly(pcm, rate, min_quiet_ms=MIN_TRAILING_QUIET_MS,
                 silence_level=SILENCE_LEVEL):
    """Whether this audio ends in silence rather than mid-word.

    Empty audio is not clean: nothing arrived at all, which is its own
    failure and should be retried the same way.
    """
    if not pcm:
        return False
    return trailing_quiet_ms(pcm, rate, silence_level) >= min_quiet_ms


# --- the sacrificial pause --------------------------------------------------
#
# Mangisoz truncates the end of its own output, and no client-side read method
# avoids it (measured 2026-09-21 and 2026-09-22). The fix is to give it
# something worthless to truncate: the expressive voice supports non-speech
# tags, and a trailing [long pause] generates silence AFTER the last word.
# Truncation then eats the silence. Measured 2026-09-22 over 6 runs of the
# same sentence:
#
#     plain              last word survived 2/6
#     + [long pause]     last word survived 6/6
#
# The pause itself is 2-6 s long, so the stream is stopped as soon as enough
# of it has arrived (TailWatcher) and what is kept is trimmed back (trim_tail).
PAUSE_TAG = "[long pause]"

#: Silence this long means the pause has begun, not a gap between sentences.
#: Sentence gaps measured under 1 s; 1.4 s was right in 6/6 runs, while 400 ms
#: stopped mid-sentence.
STOP_AFTER_QUIET_MS = 1400

#: How much of the pause to keep. The speaker needs a little dead air after
#: the last word - see LEAD_SILENCE_MS/TAIL_SILENCE_MS in mangi_movetts.py.
KEEP_TAIL_MS = 250


def with_sacrificial_pause(text, tag=PAUSE_TAG):
    """The text to ask for, with a pause the server may safely truncate."""
    text = (text or "").rstrip()
    if not text or text.endswith(tag):
        return text
    return "%s %s" % (text, tag)


def trim_tail(pcm, rate, keep_ms=KEEP_TAIL_MS):
    """Cut trailing silence back to keep_ms. Shorter tails are left alone."""
    quiet_ms = trailing_quiet_ms(pcm, rate)
    if quiet_ms <= keep_ms or rate <= 0:
        return bytes(pcm)
    drop = int((quiet_ms - keep_ms) / 1000.0 * rate) * 2
    return bytes(pcm)[:len(pcm) - drop] if drop else bytes(pcm)


class TailWatcher:
    """Says when a stream has reached the trailing pause and can be cut off.

    Fed the same chunks the caller collects. It reports True once speech has
    been heard AND `quiet_ms` of silence has followed it, which is the pause
    starting - stopping there saves the seconds the pause would otherwise take
    to arrive.
    """

    def __init__(self, rate, quiet_ms=STOP_AFTER_QUIET_MS,
                 silence_level=SILENCE_LEVEL):
        self.rate = rate
        self.quiet_ms = quiet_ms
        self.silence_level = silence_level
        self.spoke = False
        self._quiet_bytes = 0

    def feed(self, chunk):
        """True when the stream can be stopped."""
        if not chunk:
            return False
        values = samples(chunk)
        if values and max(abs(v) for v in values) > self.silence_level:
            self.spoke = True
            self._quiet_bytes = 0
            return False
        self._quiet_bytes += len(chunk)
        if not self.spoke or self.rate <= 0:
            return False
        return self._quiet_bytes >= int(self.quiet_ms / 1000.0 * self.rate) * 2


# --- the pause, on a stream that is already playing -------------------------
#
# The A3 node buffered each answer, so it could cut the pause off afterwards.
# Here the stream reaches the speaker as it arrives (mangi_movetts.py), so the
# pause has to be kept out of it on the way through. Measured on 192.168.1.84,
# 2026-09-23, four runs per voice:
#
#     kk madi   plain: 3/4 ended mid-word     + [long pause]: 1/4
#     ru maxim  + [long pause]: 2/4 ran on to 8 s of extra audio
#     en harry  + [long pause]: 2/4 ran on 1.5 s past the words
#
# so the pause is asked for in Kazakh only - see PAUSE_LANGS in mangi_movetts.


class PauseCutter:
    """Passes a stream through, minus the sacrificial pause at its end.

    Speech goes out as it arrives. Silence after speech is HELD, and released
    only when speech resumes, so a gap between sentences comes out whole and
    the trailing pause never comes out at all. Leading silence is passed
    through untouched: the speaker's opening ramp eats into it, and dropping
    it would move that loss onto the first word.

    feed() returns (audio to play now, whether the stream can be stopped).
    Once it says stop, or the stream ends, finish() gives back the first
    keep_ms of the held silence, so the last word still decays into quiet.
    Chunks from the server are not aligned to samples, so an odd byte is
    carried into the next chunk rather than splitting a sample.
    """

    def __init__(self, rate, quiet_ms=STOP_AFTER_QUIET_MS,
                 silence_level=SILENCE_LEVEL, keep_ms=KEEP_TAIL_MS):
        self._watcher = TailWatcher(rate, quiet_ms, silence_level)
        self._keep_bytes = int(keep_ms / 1000.0 * rate) * 2
        self._silence_level = silence_level
        self._carry = b""
        self._held = []
        self.spoke = False

    def _quiet(self, pcm):
        values = samples(pcm)
        return not values or max(abs(v) for v in values) <= self._silence_level

    def feed(self, chunk):
        data = self._carry + bytes(chunk or b"")
        self._carry = data[-1:] if len(data) % 2 else b""
        data = data[:len(data) - len(self._carry)]
        if not data:
            return b"", False
        if self._watcher.feed(data):
            self._held.append(data)
            return b"", True
        if self._quiet(data):
            if not self.spoke:
                return data, False
            self._held.append(data)
            return b"", False
        self.spoke = True
        out = b"".join(self._held) + data
        self._held = []
        return out, False

    def finish(self):
        """The tail to play after the last word: at most keep_ms of quiet."""
        held = b"".join(self._held)
        self._held = []
        return held[:self._keep_bytes] if self.spoke else b""
