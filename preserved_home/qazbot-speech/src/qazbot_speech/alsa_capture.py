# Adapted from earlier QazBot robot code; see NOTICE.md.
"""Reading a microphone straight off an ALSA card.

Two of them, in practice. A USB mic on the main unit, which the HAL cannot see
at all; and the analog AUX input on the INTERACTION unit, which is a different
machine (10.0.1.42) carrying the ES8388 codec that a 3.5 mm mic plugs into.
Neither reaches the pipeline on its own.

The HAL publishes two microphones: the robot's internal array, and whatever is
wired to its analog AUX input, selected with SetMicSourceRequest. A USB mic is
neither. It appears as an ALSA sound card, and `SetMicSourceRequest(1)` just
selects an empty analog input - which is why a USB mic plugged into this robot
produces a topic full of digital silence and no error anywhere.

So this reads the card directly. There is no arecord on the robot, and no
sounddevice, pyaudio or alsaaudio either, but libasound.so.2 is there, so the
capture runs through ctypes onto ALSA's own C API. Nothing to install, no root.

`plughw:` rather than `hw:`: the plugin layer converts for us, which matters
because a USB mic is typically 48 kHz stereo and everything downstream of here
assumes 16 kHz mono. Asking ALSA for 16 kHz mono makes that its problem, and it
is better at it than a hand-rolled decimator. (Measured on the Fifine on this
robot: `hw:` at 48 kHz returned -EIO on every read, `plughw:` at 16 kHz mono
read 60 chunks out of 60.)

The parts worth testing on a laptop - which card is the right one, which
channel the voice is on, what gain does to a sample - are plain functions at
the top. The ctypes below them cannot run without a sound card.
"""
import array
import ctypes
import math
import re
import shutil
import subprocess

SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2
CHANNELS = 1
# 20 ms, the chunk size the HAL publishes and the one every gate downstream was
# tuned against.
FRAMES_PER_CHUNK = SAMPLE_RATE // 50

S16_MAX = 32767
S16_MIN = -32768

SND_PCM_STREAM_CAPTURE = 1
SND_PCM_FORMAT_S16_LE = 2
SND_PCM_ACCESS_RW_INTERLEAVED = 3


# --- the testable part ------------------------------------------------------

def parse_cards(text: str):
    """[(index, id, description)] out of /proc/asound/cards.

    The file pairs a terse line with an indented description:

        0 [HDA            ]: tegra-hda - NVIDIA Jetson Orin NX HDA
                             NVIDIA Jetson Orin NX HDA at 0x3518000 irq 121
        1 [Microphone     ]: USB-Audio - fifine Microphone
                             fifine Microphone at usb-3610000.usb-3.1
    """
    cards = []
    for line in text.splitlines():
        match = re.match(r"\s*(\d+)\s*\[([^\]]*)\]\s*:\s*(.*)", line)
        if match:
            index, ident, rest = match.groups()
            cards.append((int(index), ident.strip(), rest.strip()))
    return cards


def find_card(text: str, wanted: str):
    """The index of the card whose id or description mentions `wanted`.

    Matched by name rather than pinned to a number because card numbering is
    an enumeration order, not a property of the hardware: unplug the mic at a
    conference, plug it back in, and `hw:1` can be something else entirely.
    Returns None when nothing matches, which the node reports rather than
    guessing at a card and recording the wrong room.
    """
    needle = (wanted or "").strip().lower()
    if not needle:
        return None
    for index, ident, description in parse_cards(text):
        if needle in ident.lower() or needle in description.lower():
            return index
    return None


def pick_channel(pcm: bytes, channels: int, index: int) -> bytes:
    """One channel out of an interleaved S16LE frame.

    Which channel matters more than it sounds. The AUX mic on this robot
    arrives on the RIGHT channel of the ES8388 codec - measured 2026-09-15,
    ch0 at -74.7 dBFS peak and ch1 at -2.3 dBFS with the same voice - so
    anything that assumes channel 0 records silence and reports nothing wrong.
    """
    if channels <= 1:
        return pcm
    if index < 0 or index >= channels:
        raise ValueError(f"channel {index} out of range for {channels} channels")
    unit = channels * SAMPLE_WIDTH
    if unit <= 0 or len(pcm) < unit:
        return b""
    out = bytearray()
    start = index * SAMPLE_WIDTH
    for base in range(0, len(pcm) - unit + 1, unit):
        out += pcm[base + start: base + start + SAMPLE_WIDTH]
    return bytes(out)


def loudest_channel(pcm: bytes, channels: int):
    """Which channel actually carries a voice, and every channel's level.

    Used to tell an operator where the mic is instead of making them guess:
    a mic wired to the wrong ring of a TRRS plug, or a codec routed to the
    other input, looks identical to a dead mic from channel 0.
    """
    levels = [dbfs(rms(pick_channel(pcm, channels, c)))
              for c in range(max(1, channels))]
    best = max(range(len(levels)), key=lambda c: levels[c])
    return best, levels


def rms(pcm: bytes) -> float:
    """Loudness of one S16LE chunk, in raw counts.

    Deliberately not imported from wake_audio: this module runs on the
    interaction unit too, where only these two files are installed.
    """
    if not pcm:
        return 0.0
    samples = array.array("h")
    samples.frombytes(pcm[: len(pcm) - (len(pcm) % SAMPLE_WIDTH)])
    if not len(samples):
        return 0.0
    return (sum(s * s for s in samples) / len(samples)) ** 0.5


def gain_factor(gain_db: float) -> float:
    return 10.0 ** (gain_db / 20.0)


def apply_gain(pcm: bytes, gain_db: float) -> bytes:
    """Amplify S16LE audio, clipping rather than wrapping.

    A USB mic with its dial low can arrive 30 dB quieter than the internal
    array, far below any threshold tuned for the latter. Digital gain is the
    poor relation of turning the dial up - it lifts the noise with the voice -
    but it is what there is when nobody can reach the robot.

    Clipping matters: S16 arithmetic that wraps turns a loud vowel into a burst
    of noise at full scale, which reads as speech to every gate downstream.
    """
    if not pcm or not gain_db:
        return pcm
    factor = gain_factor(gain_db)
    samples = array.array("h")
    samples.frombytes(pcm)
    for i, sample in enumerate(samples):
        scaled = int(sample * factor)
        samples[i] = S16_MAX if scaled > S16_MAX else (
            S16_MIN if scaled < S16_MIN else scaled)
    return samples.tobytes()


def dbfs(level: float) -> float:
    """Duplicated deliberately from wake_audio, which this must not import:
    that module is the wake node's, and this one runs in a node that must come
    up even if the wake node is not installed."""
    if level <= 0.0:
        return -96.0
    return max(-96.0, 20.0 * math.log10(level / 32768.0))


# --- the part that needs a sound card ---------------------------------------

class AlsaError(OSError):
    pass


class ArecordCapture:
    """Capture by piping `arecord`, for cards where the library path fails.

    On the interaction unit's ES8388 the ctypes path below opens, configures
    and reads at exactly the right rate - and returns digital silence from the
    second second onwards. Measured 2026-09-15, per second, while the robot was
    speaking into the room:

        t=0s  peak -11.1 dBFS      <- the buffer that was already in the card
        t=1s  peak -90.7 dBFS
        t=2s  peak -90.7 dBFS      ... and so on for ever

    `arecord` on the same device at the same moment records the speech
    properly (-35 dBFS rms over a ten second file, which no transient can
    fake). Rather than keep guessing at which of set_params, prepare or start
    the Rockchip driver dislikes, this uses the tool that works and reads its
    raw output. One subprocess, no ALSA API of our own.
    """

    def __init__(self, device="plughw:1,0", rate=SAMPLE_RATE,
                 channels=CHANNELS, binary="arecord"):
        self.device = device
        self.rate = rate
        self.channels = channels
        self._frame_bytes = SAMPLE_WIDTH * channels
        self._chunk_bytes = FRAMES_PER_CHUNK * self._frame_bytes
        cmd = [binary, "-D", device, "-f", "S16_LE", "-r", str(rate),
               "-c", str(channels), "-t", "raw", "-q"]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                bufsize=0)
        except OSError as e:
            raise AlsaError(f"starting {' '.join(cmd)}: {e}")

    def read(self):
        """One chunk, or None when arecord has stopped.

        Reads exactly one chunk's worth so the caller's timing is unchanged
        from the ctypes path; arecord paces itself from the card.
        """
        if self._proc.poll() is not None:
            return None
        try:
            data = self._proc.stdout.read(self._chunk_bytes)
        except (OSError, ValueError):
            return None
        if not data or len(data) < self._frame_bytes:
            return None
        return data

    def stderr_tail(self) -> str:
        """Whatever arecord complained about, for a message worth reading."""
        if self._proc.poll() is None:
            return ""
        try:
            return (self._proc.stderr.read() or b"").decode(errors="replace").strip()
        except Exception:
            return ""

    def close(self):
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except Exception:
                self._proc.kill()
        for pipe in (self._proc.stdout, self._proc.stderr):
            try:
                pipe.close()
            except Exception:
                pass


def open_capture(device, channels=CHANNELS, rate=SAMPLE_RATE, backend="auto"):
    """The capture for this machine.

    `arecord` where it exists, because it is the one that works on the
    interaction unit's codec; the ctypes reader otherwise, which is what the
    main unit has (no alsa-utils there) and which the USB mic is happy with.
    """
    want = (backend or "auto").strip().lower()
    have_arecord = shutil.which("arecord")
    if want == "arecord" or (want == "auto" and have_arecord):
        if not have_arecord:
            raise AlsaError("arecord backend asked for, but arecord is not installed")
        return ArecordCapture(device, rate=rate, channels=channels)
    return AlsaCapture(device, rate=rate, channels=channels)


class AlsaCapture:
    """One ALSA capture stream, opened at 16 kHz mono S16LE.

    Not thread-safe; the node reads it from one thread and nothing else
    touches it.
    """

    def __init__(self, device="plughw:1,0", rate=SAMPLE_RATE,
                 channels=CHANNELS, latency_us=100000):
        self._lib = ctypes.CDLL("libasound.so.2")
        self._lib.snd_strerror.restype = ctypes.c_char_p
        self.device = device
        self.rate = rate
        self.channels = channels
        self._handle = ctypes.c_void_p()
        self._check(self._lib.snd_pcm_open(
            ctypes.byref(self._handle), device.encode(),
            SND_PCM_STREAM_CAPTURE, 0), f"opening {device}")
        self._check(self._lib.snd_pcm_set_params(
            self._handle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
            channels, rate, 1, ctypes.c_uint(latency_us)),
            f"configuring {device} for {rate} Hz, {channels} ch")
        # A shallow ring buffer on purpose. Half a second of slack lets the
        # card run ahead of a busy reader and then deliver a burst, which
        # downstream reads as a pause followed by speeded-up speech; 100 ms
        # keeps the stream close to real time and turns a reader that cannot
        # keep up into an honest overrun the node can count.
        self._lib.snd_pcm_prepare(self._handle)
        self._lib.snd_pcm_start(self._handle)
        self._frame_bytes = SAMPLE_WIDTH * channels
        self._buffer = ctypes.create_string_buffer(
            FRAMES_PER_CHUNK * self._frame_bytes)

    def _check(self, rc, what):
        if rc < 0:
            raise AlsaError(f"{what}: {self._lib.snd_strerror(rc).decode()}")
        return rc

    def read(self):
        """One chunk, or None when the stream hiccuped and was recovered.

        An overrun is normal on a busy robot and is not worth a log line every
        time; it is only worth one when it never stops, which the node counts.
        """
        got = self._lib.snd_pcm_readi(
            self._handle, self._buffer, FRAMES_PER_CHUNK)
        if got < 0:
            self._lib.snd_pcm_recover(self._handle, got, 1)
            return None
        return self._buffer.raw[: got * self._frame_bytes]

    def close(self):
        if self._handle:
            self._lib.snd_pcm_close(self._handle)
            self._handle = ctypes.c_void_p()


def read_cards(path="/proc/asound/cards") -> str:
    try:
        with open(path, "r") as handle:
            return handle.read()
    except OSError:
        return ""
