"""Mic sources. Each is an async iterator of (t_seconds, pcm_bytes) frames of about 100 ms.

`ReplayMic` plays a recorded wav at real-time pace (laptop testing). `pick_mic` applies the priority
Fifine USB, then Maono, then the robot's internal array; the real capture classes are ported from the
Digital Bridge `usb_mic` / HAL reader when the robot audio path is built.
"""
import asyncio
import time
import wave

PRIORITY = ("fifine", "maono", "internal")


class ReplayMic:
    def __init__(self, wav_path, lead_silence=0.5, tail_silence=3.0, frame_sec=0.1, realtime=True):
        w = wave.open(wav_path, "rb")
        self.rate, self.pcm = w.getframerate(), w.readframes(w.getnframes())
        self.lead, self.tail, self.frame, self.realtime = lead_silence, tail_silence, frame_sec, realtime

    def __aiter__(self):
        return self._gen()

    async def _gen(self):
        step = int(self.rate * self.frame) * 2
        data = bytes(int(self.rate * self.lead) * 2) + self.pcm + bytes(int(self.rate * self.tail) * 2)
        t0 = time.monotonic()
        for n, i in enumerate(range(0, len(data), step), 1):
            yield n * self.frame, data[i:i + step]
            if self.realtime:
                await asyncio.sleep(max(0.0, t0 + n * self.frame - time.monotonic()))


def pick_mic(available):
    """`available` maps a mic name (fifine / maono / internal) to a ready mic object. Returns (name, mic)
    for the highest-priority one present, or (None, None)."""
    for name in PRIORITY:
        if name in available and available[name] is not None:
            return name, available[name]
    return None, None


class MicStalled(RuntimeError):
    """The capture produced no audio for `stall_sec`; the caller falls back to the next mic."""


class AlsaMic:
    """A USB mic read straight off its ALSA card (Fifine, Maono), as 100 ms frames.

    `capture` is an alsa_capture-style object: `read()` returns a 20 ms chunk (or None after an overrun) and
    `close()`. Reads block, so they run in a worker thread. If a read takes longer than `stall_sec`, the mic
    is closed and MicStalled is raised (Digital Bridge: the USB reader reopened the card after 5 s without
    reads and fell back to the robot array after 10 s of failures).
    """

    def __init__(self, capture, frame_chunks=5, stall_sec=5.0, clock=time.monotonic):
        self.capture, self.frame_chunks, self.stall_sec, self.clock = capture, frame_chunks, stall_sec, clock

    def __aiter__(self):
        return self._gen()

    async def _gen(self):
        loop = asyncio.get_event_loop()
        t0, n, buf = self.clock(), 0, []
        try:
            while True:
                try:
                    chunk = await asyncio.wait_for(loop.run_in_executor(None, self.capture.read), self.stall_sec)
                except asyncio.TimeoutError:
                    raise MicStalled("no audio for %.1f s" % self.stall_sec)
                if not chunk:
                    continue
                buf.append(chunk)
                if len(buf) >= self.frame_chunks:
                    n += 1
                    yield self.clock() - t0, b"".join(buf)
                    buf = []
        finally:
            self.capture.close()


def discover(cards_text, card_names=("fifine", "maono")):
    """Which named USB mics are present in /proc/asound/cards text: {name: card index}."""
    from .alsa_capture import find_card
    found = {}
    for name in card_names:
        idx = find_card(cards_text, name)
        if idx is not None:
            found[name] = idx
    return found


class AutoMic:
    """One mic stream that follows the plug: it uses the best mic present (fifine > maono > internal) and
    re-scans every `rescan_sec`, so a Fifine plugged in after boot takes over without a restart, and an
    unplugged or stalled one falls back to the next.

    `scan()` returns the mic names present now; `make(name)` builds that mic; `on_switch(name)` is told
    whenever the active mic changes (the service swaps the VAD profile there). Frame times stay monotonic
    across switches. A mic that fails to open or stalls is skipped for `retry_sec`.
    """

    def __init__(self, scan, make, on_switch=None, rescan_sec=5.0, retry_sec=20.0, clock=time.monotonic,
                 log=None):
        self.scan, self.make, self.on_switch = scan, make, on_switch
        self.rescan_sec, self.retry_sec, self.clock, self.log = rescan_sec, retry_sec, clock, log
        self.active = None
        self._bad = {}          # name -> clock time it may be tried again

    def __aiter__(self):
        return self._gen()

    def _best(self):
        now = self.clock()
        present = set(self.scan())
        for name in PRIORITY:
            if name in present and self._bad.get(name, 0.0) <= now:
                return name
        return None

    def _say(self, msg):
        if self.log:
            self.log(msg)

    async def _gen(self):
        base = 0.0
        while True:
            name = self._best()
            if name is None:
                await asyncio.sleep(1.0)
                continue
            try:
                it = self.make(name).__aiter__()
            except Exception as err:
                self._bad[name] = self.clock() + self.retry_sec
                self._say("mic %s would not open: %r" % (name, err))
                continue
            if name != self.active:
                self._say("mic: %s -> %s" % (self.active, name))
                self.active = name
                if self.on_switch:
                    self.on_switch(name)
            last_t, next_scan = 0.0, self.clock() + self.rescan_sec
            try:
                async for t, pcm in it:
                    last_t = t
                    yield base + t, pcm
                    if self.clock() >= next_scan:
                        next_scan = self.clock() + self.rescan_sec
                        if self._best() != name and PRIORITY.index(self._best() or "internal") < PRIORITY.index(name):
                            break              # a better mic appeared: switch
            except (MicStalled, OSError, RuntimeError) as err:
                self._bad[name] = self.clock() + self.retry_sec
                self._say("mic %s failed (%s): trying the next one" % (name, err))
            finally:
                await it.aclose()
                base += last_t + 0.1


class InternalMic:
    """The robot's own mic array from the HAL (/aima/hal/audio/capture), voice channel only, as 100 ms frames.

    The HAL publishes the mic channels plus loudspeaker-reference channels interleaved (S16LE); the voice is
    channel 0 (Digital Bridge QBOT_AUDIO_CHANNEL). It hears the hall rather than the guest, so it is last in
    the priority list. `subscribe(callback)` is provided by robot.ros_io; callback(pcm_interleaved, channels)
    runs on a ROS thread, so frames are handed over with call_soon_threadsafe.
    """

    def __init__(self, subscribe, rate=16000, voice_channel=0, frame_sec=0.1, stall_sec=5.0, clock=time.monotonic):
        self.subscribe, self.rate, self.voice_channel = subscribe, rate, voice_channel
        self.frame_bytes, self.stall_sec, self.clock = int(rate * frame_sec) * 2, stall_sec, clock

    def __aiter__(self):
        return self._gen()

    async def _gen(self):
        from .alsa_capture import pick_channel
        loop, q = asyncio.get_event_loop(), asyncio.Queue()

        def on_audio(pcm, channels):
            voice = pick_channel(pcm, channels, self.voice_channel) if channels > 1 else pcm
            loop.call_soon_threadsafe(q.put_nowait, voice)

        unsubscribe = self.subscribe(on_audio)
        t0, buf = self.clock(), b""
        try:
            while True:
                try:
                    buf += await asyncio.wait_for(q.get(), self.stall_sec)
                except asyncio.TimeoutError:
                    raise MicStalled("no HAL audio for %.1f s" % self.stall_sec)
                while len(buf) >= self.frame_bytes:
                    yield self.clock() - t0, buf[:self.frame_bytes]
                    buf = buf[self.frame_bytes:]
        finally:
            if unsubscribe:
                unsubscribe()
