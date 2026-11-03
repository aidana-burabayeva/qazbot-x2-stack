"""Run the speech service.

  python -m qazbot_speech --env .env --replay clip.wav        laptop: replay a wav as the mic, log replies
  python -m qazbot_speech --env .env --replay clip.wav --robot  on a robot: speak through the HAL
  python -m qazbot_speech --env .env --robot                   on a robot: USB mic (Fifine, Maono) else internal

Settings that operators change live (wake names, idle sleep) come from a small JSON file, re-read every turn:
{"wake_names": ["Азамат"], "idle_sleep": 15}. The operator site writes it; this service never does.
"""
import argparse
import asyncio
import json
import os
import socket
import time

from aiohttp import web

from . import api as api_mod
from .alsa_capture import open_capture, read_cards
from .brief import BriefFile
from .mangisoz import MangisozSTT, MangisozTTS
from .mics import AlsaMic, AutoMic, InternalMic, MicStalled, ReplayMic, discover
from .openrouter_llm import OpenRouterLLM
from .qa import FastAnswers
from .race import DualSTT, RaceTTS
from .robot import HalSpeaker, ros_io
from .service import Service
from .session import Session
from .soniox_stt import SonioxSTT
from .soniox_tts import SonioxTTS
from .telemetry import Telemetry
from .tools import Weather
from .turn import Turn
from .vad import apply_profile, vad_for


# 127.0.0.2, not 127.0.0.1: on the robots Tailscale forwards every tailnet connection to 127.0.0.1, so a
# service on 127.0.0.1 is open to the whole tailnet without a password (guest transcripts, /api/say,
# /api/clear-memory). The site and the dance page already use 127.0.0.2 for the same reason.
DEFAULT_HOST = "127.0.0.2"


def check_host(host):
    """Refuse any API bind address outside loopback, and 127.0.0.1 itself (the Tailscale-forwarded one)."""
    import ipaddress
    ip = ipaddress.ip_address(host)
    if not ip.is_loopback or str(ip) == "127.0.0.1":
        raise SystemExit("refusing to bind the speech API to %s: use an address in 127.0.0.0/8 other than "
                         "127.0.0.1 (default %s)" % (host, DEFAULT_HOST))
    return str(ip)


def load_env(path):
    if path and os.path.exists(path):
        for line in open(path, encoding="utf-8"):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip())


class LogSpeaker:
    """Laptop speaker: counts audio, prints nothing audible."""
    is_speaking = False

    def __init__(self):
        self.bytes, self.gestures = 0, []

    async def play(self, chunk):
        self.bytes += len(chunk)

    async def gesture(self, code):
        self.gestures.append(code)
        return True

    async def drain(self):
        pass

    def interrupt(self):
        pass


class Config:
    def __init__(self, path):
        self.path, self._m, self.data = path, None, {}

    def get(self):
        try:
            st = os.stat(self.path)
            m = (st.st_mtime_ns, st.st_size)
            if m != self._m:
                self.data, self._m = json.load(open(self.path, encoding="utf-8")), m
        except (OSError, ValueError):
            pass
        return self.data


def build_mic(args, subscribe_capture=None, on_switch=None, log=None):
    """Returns (mic_factory, first_mic_name). On a robot the factory builds an AutoMic that follows the plug:
    a Fifine connected after boot takes over within a few seconds, an unplugged one falls back."""
    if args.replay:
        return lambda: ReplayMic(args.replay, tail_silence=args.tail), "replay"

    def scan():
        names = list(discover(read_cards()))
        if subscribe_capture:
            names.append("internal")
        return names

    def make(name):
        if name == "internal":
            return InternalMic(subscribe_capture)
        return AlsaMic(open_capture("plughw:%d,0" % discover(read_cards())[name]))

    first = scan()
    if not first:
        raise SystemExit("No USB mic (fifine/maono) found; run with --robot to use the internal array.")
    auto = AutoMic(scan, make, on_switch=on_switch, log=log)
    return (lambda: auto), next(n for n in ("fifine", "maono", "internal") if n in first)


async def amain(args):
    load_env(args.env)
    e = os.environ
    archive = None
    if args.robot and e.get("QAZBOT_CONVO_LOG", "1") != "0":     # robots keep a conversation log; --replay and laptops do not
        archive = e.get("QAZBOT_CONVO_DIR") or os.path.join(os.path.expanduser("~"), "qazbot_logs")
    tel = Telemetry(archive_dir=archive, robot=socket.gethostname())
    io = ros_io() if args.robot else None
    holder = {}
    log = tel.log

    def on_mic_switch(name):
        tel.mic["active"] = name
        if "svc" in holder:
            apply_profile(holder["svc"].vad, name)

    mic_factory, mic_name = build_mic(args, io[4] if io else None, on_switch=on_mic_switch, log=log)
    tel.mic["active"] = mic_name
    rate = 24000 if args.replay else 16000
    if args.replay:
        import wave
        rate = wave.open(args.replay).getframerate()
    soniox_stt = SonioxSTT(e["SONIOX_API_KEY"], sample_rate=rate)
    stt = DualSTT(soniox_stt, MangisozSTT(e["MANGISOZ_API_KEY"], e["MANGISOZ_BASE_URL"], sample_rate=rate)) \
        if e.get("MANGISOZ_API_KEY") else soniox_stt
    soniox_tts = SonioxTTS(e["SONIOX_API_KEY"])
    if e.get("MANGISOZ_API_KEY"):
        mangi = MangisozTTS(e["MANGISOZ_API_KEY"], e["MANGISOZ_BASE_URL"])
        # The same voice twice first (a second identical request after 0.9 s), and Soniox's English-accented
        # voice only after 2.5 s or a failure: a voice that changes mid-conversation sounds like a foreign accent.
        tts = RaceTTS([mangi, mangi, soniox_tts], schedule=(0.9, 2.5))
    else:
        tts = soniox_tts
    llm = OpenRouterLLM(e["OPENROUTER_API_KEY"])
    if args.robot:
        publish, motion, focus, release, _, _ = io
        speaker = HalSpeaker(publish, motion, pkg_name="qazbot_speech", focus=focus, release=release)
    else:
        speaker = LogSpeaker()
    home = os.path.expanduser("~")
    # The brief path is the contract with the operator site: QBOT_VENUE_BRIEF_FILE is what dance-bind sets.
    brief = BriefFile(e.get("QAZBOT_BRIEF_FILE") or e.get("QBOT_VENUE_BRIEF_FILE")
                      or os.path.join(home, "qbot_knowledge", "brief.md"))
    cfg = Config(e.get("QAZBOT_CONFIG") or os.path.join(home, ".config", "qazbot_speech", "config.json"))
    session = Session(time.monotonic)
    turn = Turn(stt, llm, tts, speaker, tools=[Weather()])
    svc = Service(mic_factory, stt, turn, session, tel, persistent_mic=not args.replay, vad=vad_for(mic_name),
                  log=log)

    holder["svc"] = svc

    async def say(text, lang):
        await turn._speak(text, lang or "ru", lambda *a: None)
        if hasattr(speaker, "drain"):
            await speaker.drain()

    async def self_test():
        res = {}
        for name, coro in (("llm", llm.warm()), ("tts", soniox_tts.warm())):
            t0 = time.monotonic()
            await coro
            res[name] = round((time.monotonic() - t0) * 1000)
            tel.set_health(name, True, ms=res[name])
        return res

    runner = web.AppRunner(api_mod.make_app(tel, on_clear_memory=svc.clear_memory, on_self_test=self_test,
                                            on_say=say, config=cfg.get))
    await runner.setup()
    await web.TCPSite(runner, check_host(args.host), args.port).start()
    await self_test()
    try:
        while True:
            c = cfg.get()
            session.names, session.idle_sleep = tuple(c.get("wake_names", ())), float(c.get("idle_sleep", 15))
            if brief.text != turn.brief:
                turn.fast_answers = FastAnswers(brief.text)      # the sheet's own answers skip the LLM
            turn.brief = brief.text
            for key, attr in (("wake_peak_dbfs", "wake_peak"), ("awake_peak_dbfs", "awake_peak")):
                if key in c:                     # per-robot tuning from the operator site's config file
                    setattr(svc.vad, attr, float(c[key]))
            try:
                if not args.replay:
                    await svc.run()              # full duplex: never returns
                rec = await svc.step()
            except MicStalled as err:
                tel.set_health("mic", False, note=str(err))
                print("mic stalled:", err, flush=True)
                await svc.close()
                await asyncio.sleep(1)
                continue
            except asyncio.CancelledError:
                raise
            except Exception as err:   # one bad turn must not end the service
                import traceback
                traceback.print_exc()
                tel.set_health("turn", False, note=repr(err)[:120])
                await svc.close()
                await asyncio.sleep(1)
                continue
            if rec:
                print("[%s] %s -> %s (first audio %.2fs)" % (rec["lang"], rec["text"], rec["reply"][:80],
                                                           rec["first_audio_s"] or -1), flush=True)
                if hasattr(speaker, "drain"):
                    await speaker.drain()
            if args.replay:
                break
    finally:
        await svc.close()
        await runner.cleanup()


def main():
    p = argparse.ArgumentParser(prog="qazbot_speech")
    p.add_argument("--env", default=".env")
    p.add_argument("--replay", help="wav file played as the mic (laptop testing)")
    p.add_argument("--tail", type=float, default=4.0, help="silence appended after the replay clip, seconds")
    p.add_argument("--robot", action="store_true", help="use the robot's HAL speaker and arm (needs ROS 2)")
    p.add_argument("--port", type=int, default=18100)
    p.add_argument("--host", default=DEFAULT_HOST,
                   help="API bind address; must be in 127.0.0.0/8 (default %s)" % DEFAULT_HOST)
    loop = asyncio.get_event_loop()
    task = asyncio.ensure_future(amain(p.parse_args()))
    try:
        import signal
        for sig in (signal.SIGTERM, signal.SIGINT):     # systemd stop / Ctrl-C: leave the mic and ROS cleanly
            loop.add_signal_handler(sig, task.cancel)
    except (NotImplementedError, ImportError):          # Windows laptop
        pass
    try:
        loop.run_until_complete(task)
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    main()
