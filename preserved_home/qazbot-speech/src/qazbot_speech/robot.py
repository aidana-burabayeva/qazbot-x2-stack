"""The robot-side Speaker: audio to the HAL, gestures to the motion controller.

The logic that matters is pure and testable on a laptop: pacing, interrupt flush, gesture area choice.
The ROS 2 glue (`ros_io`) imports rclpy and aimdk_msgs lazily and only works on the robot.

Facts from Digital Bridge (mangi_movetts.py, tts_flush.py), kept:
  * Audio goes to /aima/hal/audio/playback as AudioPlayback{pkg_name, token_id, info, data}.
  * Stopping the TTS stream does not stop the speaker: what the HAL already holds plays to the end. The HAL
    keys its buffer by token_id, so a fresh, never-repeated token with a short silence cuts it off.
  * Audio is handed over paced (about 1 s ahead of real time), not at network speed, so that a flush has
    little left to cut.
"""
import asyncio
import itertools
import time

from . import gestures

_counter = itertools.count()


def flush_token(pkg_name):
    """A token_id never used before in this process (a repeated token is a stop the HAL ignores)."""
    return "%s-flush-%d-%d" % (pkg_name, int(time.monotonic() * 1000), next(_counter))


class HalSpeaker:
    def __init__(self, publish, motion, rate=24000, channels=1, max_lead_ms=1000.0, pkg_name="qazbot_speech",
                 clock=time.monotonic, sleep=asyncio.sleep, allowed_motions=None, focus=None, release=None, prime_ms=400.0, lead_ms=150.0, tail_ms=350.0):
        """publish(pkg_name, token_id, rate, channels, pcm_bytes) hands audio to the HAL;
        motion(code, area) asks the motion controller. Both are provided by `ros_io` on the robot."""
        self._publish, self._motion = publish, motion
        self._focus, self._release = focus, release   # audio focus: asked for at the start of a stretch of speech
        self.rate, self.channels, self.max_lead_ms = rate, channels, max_lead_ms
        self.pkg_name, self.clock, self.sleep = pkg_name, clock, sleep
        self.allowed = None if allowed_motions is None else set(allowed_motions)
        self._first_at = None
        self._handed_ms = 0.0
        self.is_speaking = False
        self.prime_ms = prime_ms        # audio held back at the start of a stretch so the player does not run dry
        self._primed = b""
        self._carry = b""             # an odd trailing byte, held until the next chunk completes the sample
        self.lead_ms, self.tail_ms = lead_ms, tail_ms   # dead air around speech: the speaker amp ramps up and down

    @property
    def _bytes_per_ms(self):
        return self.rate * self.channels * 2 / 1000.0

    def _lead_ms(self):
        if self._first_at is None:
            return 0.0
        return self._handed_ms - (self.clock() - self._first_at) * 1000.0

    async def play(self, chunk):
        """Hand a chunk to the HAL, waiting while more than max_lead_ms is already queued ahead.

        At the start of a stretch of speech, the first `prime_ms` of audio is held and sent in one piece: the
        HAL log showed the player flipping between PLAYING and STOPPED when chunks trickled in one by one.
        """
        # Samples are 2 bytes. Network chunks come in any size; an odd chunk shifts every later sample by one byte
        # and plays as a burst of static ("bsht brsht"), so only whole samples go to the HAL.
        data = self._carry + bytes(chunk)
        self._carry = data[-1:] if len(data) % 2 else b""
        chunk = data[:len(data) - len(self._carry)]
        if not chunk:
            return
        if self._first_at is None and self.prime_ms:
            self._primed += bytes(chunk)
            if len(self._primed) / self._bytes_per_ms < self.prime_ms:
                return
            chunk, self._primed = self._primed, b""
        while self.max_lead_ms and self._lead_ms() >= self.max_lead_ms:
            await self.sleep(0.02)
        if self._first_at is None or self._lead_ms() <= -50:   # the speaker drained: a new stretch starts
            self._first_at, self._handed_ms = self.clock(), 0.0
            if self._focus:
                self._focus()
            chunk = self._silence(self.lead_ms) + bytes(chunk)   # the opening ramp eats this, not the first word
        self.is_speaking = True
        self._handed_ms += len(chunk) / self._bytes_per_ms
        self._publish(self.pkg_name, self.pkg_name, self.rate, self.channels, bytes(chunk))

    async def flush(self):
        """The current TTS stream ended: send whatever priming still holds back."""
        if self._primed:
            chunk, self._primed = self._primed, b""
            saved, self.prime_ms = self.prime_ms, 0
            try:
                await self.play(chunk)
            finally:
                self.prime_ms = saved

    def _silence(self, ms):
        return bytes(int(self.rate * self.channels * 2 * ms / 1000)) if ms else b""

    async def drain(self):
        """Wait until the speaker has actually played what it was handed (plus a short silent tail, so the
        amp ramping down cuts silence, not the last syllable)."""
        if self.is_speaking and self._first_at is not None and self.tail_ms:
            self._handed_ms += self.tail_ms
            self._publish(self.pkg_name, self.pkg_name, self.rate, self.channels, self._silence(self.tail_ms))
        while self._lead_ms() > 0:
            await self.sleep(0.02)
        self.is_speaking = False
        if self._release and self._first_at is not None:
            self._release()
            self._first_at, self._handed_ms = None, 0.0

    def interrupt(self, silence_ms=100):
        """Cut the robot off mid-answer: a fresh token with a short silence replaces the HAL's queue."""
        silence = bytes(int(self.rate * self.channels * 2 * silence_ms / 1000))
        self._primed, self._carry = b"", b""
        self._publish(self.pkg_name, flush_token(self.pkg_name), self.rate, self.channels, silence)
        self._first_at, self._handed_ms, self.is_speaking = None, 0.0, False

    async def gesture(self, code):
        if self.allowed is not None and code not in self.allowed:
            return False
        self._motion(code, gestures.area_for(code))
        return True


def ros_io(node_name="qazbot_speech"):
    """Create the ROS 2 bindings. Robot only. Returns (publish, motion, shutdown)."""
    import threading

    import rclpy
    from aimdk_msgs.msg import (AudioData, AudioInfo, AudioPlayback, McControlArea, McPresetMotion,
                                RequestHeader)
    from aimdk_msgs.msg import FocusRequester
    from aimdk_msgs.srv import AbandonAudioFocus, RequestAudioFocus, SetMcPresetMotion
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = rclpy.create_node(node_name)
    audio_pub = node.create_publisher(AudioPlayback, "/aima/hal/audio/playback", 10)
    motion_client = node.create_client(SetMcPresetMotion, "/aimdk_5Fmsgs/srv/SetMcPresetMotion")
    focus_client = node.create_client(RequestAudioFocus, "/aimdk_5Fmsgs/srv/RequestAudioFocus")
    abandon_client = node.create_client(AbandonAudioFocus, "/aimdk_5Fmsgs/srv/AbandonAudioFocus")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def publish(pkg_name, token_id, rate, channels, pcm):
        msg = AudioPlayback()
        msg.pkg_name, msg.token_id = pkg_name, token_id
        info, data = AudioInfo(), AudioData()
        info.channels, info.sample_rate = channels, rate
        data.data = list(pcm)
        msg.info, msg.data = info, data
        audio_pub.publish(msg)

    def motion(code, area):
        if not motion_client.service_is_ready():
            return
        req = SetMcPresetMotion.Request()
        req.header = RequestHeader()
        req.header.stamp = node.get_clock().now().to_msg()
        m, a = McPresetMotion(), McControlArea()
        m.value, a.value = int(code), int(area)
        req.motion, req.area, req.interrupt, req.play_timestamp = m, a, True, 0
        motion_client.call_async(req)   # fire and forget: a gesture is not worth blocking a reply on

    def subscribe_capture(callback):
        """callback(pcm_interleaved_s16le, channels) for every HAL capture message; returns an unsubscribe."""
        from aimdk_msgs.msg import AudioCapture
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

        def on_msg(msg):
            channels = msg.mic_channels + msg.ref_channels
            if channels:
                callback(bytes(msg.data.data), channels)
        sub = node.create_subscription(AudioCapture, "/aima/hal/audio/capture", on_msg,
                                       QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                                                  reliability=QoSReliabilityPolicy.BEST_EFFORT))
        return lambda: node.destroy_subscription(sub)

    def _requester():
        r = FocusRequester()
        r.pkg_name, r.priority = "qazbot_speech", 6
        return r

    def focus():
        if focus_client.service_is_ready():     # never wait: a speaking turn must not stall on a service
            req = RequestAudioFocus.Request()
            req.focus_requester = _requester()
            focus_client.call_async(req)

    def release():
        if abandon_client.service_is_ready():
            req = AbandonAudioFocus.Request()
            req.focus_requester = _requester()
            abandon_client.call_async(req)

    def shutdown():
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    return publish, motion, focus, release, subscribe_capture, shutdown
