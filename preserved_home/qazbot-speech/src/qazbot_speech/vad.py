"""Energy VAD with a room-calibrated threshold.

Digital Bridge settings that worked with the Fifine hand mic (env_fifine.sh): threshold auto-calibrated
from the room, clamped to -31..-12 dBFS, 6 dB margin over the noise, 3 s noise window. Samruk showed what a
fixed or badly tuned gate costs: chatter held turns open for 10-20 s, quiet first questions came 10 s late.
"""
import array
import collections
import math


def dbfs(pcm):
    """Level of 16-bit little-endian mono PCM in dBFS (-96 for silence)."""
    a = array.array("h")
    a.frombytes(pcm[:len(pcm) // 2 * 2])
    if not a:
        return -96.0
    rms = math.sqrt(sum(x * x for x in a) / len(a))
    return 20 * math.log10(rms / 32768.0) if rms > 0 else -96.0


class EnergyVad:
    def __init__(self, floor_dbfs=-31.0, ceil_dbfs=-12.0, margin_db=6.0, window_frames=30,
                 wake_peak_dbfs=-24.0, awake_peak_dbfs=-34.0):
        # An utterance whose loudest frame is below wake_peak (robot asleep) or awake_peak (in a conversation) is
        # background talk (a video, people nearby), not someone speaking into the mic. Measured 2026-10-15: the
        # guest's real quiet speech peaks at -26 to -29 dBFS and background talk at -25 to -31, so a level cannot
        # tell them apart: it only guards WAKING UP (-24); inside a conversation (-34, below the VAD gate) it is off.
        self.wake_peak, self.awake_peak = wake_peak_dbfs, awake_peak_dbfs
        self.floor, self.ceil, self.margin = floor_dbfs, ceil_dbfs, margin_db
        self.noise = collections.deque(maxlen=window_frames)   # levels of recent non-speech frames
        self.threshold = floor_dbfs

    def update_threshold(self):
        if len(self.noise) >= 5:
            median = sorted(self.noise)[len(self.noise) // 2]
            self.threshold = min(self.ceil, max(self.floor, median + self.margin))
        return self.threshold

    def is_speech(self, level_dbfs):
        speech = level_dbfs >= self.threshold
        if not speech:
            self.noise.append(level_dbfs)   # only quiet frames teach the room level
            self.update_threshold()
        return speech


# Gate bounds per mic. Fifine: Digital Bridge env_fifine.sh (-31..-12, 6 dB margin). Internal array: measured
# on Nagatoro 2026-10-15, a quiet room sits at -52..-48 dBFS on the voice channel and a guest is quieter than
# at a hand mic, so the floor is lower; Digital Bridge used a fixed -38 there.
PROFILES = {
    "fifine": dict(floor_dbfs=-31.0, ceil_dbfs=-12.0, margin_db=6.0),
    "maono": dict(floor_dbfs=-31.0, ceil_dbfs=-12.0, margin_db=6.0),
    "internal": dict(floor_dbfs=-42.0, ceil_dbfs=-20.0, margin_db=8.0, wake_peak_dbfs=-34.0, awake_peak_dbfs=-40.0),
    "replay": dict(floor_dbfs=-31.0, ceil_dbfs=-12.0, margin_db=6.0),
}


def vad_for(mic_name):
    return EnergyVad(**PROFILES.get(mic_name, PROFILES["fifine"]))


def apply_profile(vad, mic_name):
    """Re-tune a running gate for another mic (the Fifine was plugged in): new bounds, noise estimate reset."""
    d = EnergyVad(**PROFILES.get(mic_name, PROFILES["fifine"]))
    vad.floor, vad.ceil, vad.margin = d.floor, d.ceil, d.margin
    vad.wake_peak, vad.awake_peak = d.wake_peak, d.awake_peak
    vad.noise.clear()
    vad.threshold = d.floor
