"""Recent turns and provider health, kept in memory and served by the local API.

With `archive_dir` every turn is also appended to <archive_dir>/conversations-YYYY-MM-DD.jsonl, one JSON
object per line: what the guest said (the transcript, no audio), what the robot answered, the language,
the path (Q&A fast path, prefetch) and the stage timings. The fleet site reads these files over SSH.
Switch off with QAZBOT_CONVO_LOG=0. Old files go after `keep_days` days or past `max_mb` in total.
"""
import collections
import json
import os
import time


class Telemetry:
    def __init__(self, keep=200, clock=time.time, archive_dir=None, robot="", keep_days=90, max_mb=200):
        self.archive_dir, self.robot = archive_dir, robot
        self.keep_days, self.max_mb, self._pruned_on = keep_days, max_mb, None
        self.turns = collections.deque(maxlen=keep)
        self.health = {}      # provider name -> {"ok": bool, "ms": float|None, "note": str, "at": float}
        self.mic = {"active": None, "level_dbfs": None}
        self.clock = clock
        self.lines = collections.deque(maxlen=400)    # the service's own log, served to the site

    def log(self, msg):
        """Print a log line and keep it for /api/log."""
        self.lines.append({"at": self.clock(), "msg": str(msg)})
        print("  .", msg, flush=True)

    def record_turn(self, events, audio_sec=None, extra=None):
        """`events` come from Turn.run. Stage times are relative to the end of the guest's audio."""
        by = {}
        for e in events:
            by.setdefault(e.kind, e)
        tr = by.get("transcript")
        rec = {
            "at": self.clock(),
            "text": tr.data["text"] if tr else "",
            "lang": tr.data["lang"] if tr else None,
            "stt_s": round(tr.t, 3) if tr else None,
            "first_sentence_s": round(by["sentence"].t, 3) if "sentence" in by else None,
            "first_audio_s": round(by["audio"].t, 3) if "audio" in by else None,
            "fast_path": "fast_path" in by,
            "prefetch": "prefetch_hit" in by,
            "bridge": "bridge" in by,
            "gesture": by["gesture"].data if "gesture" in by else None,
            "reply": " ".join(e.data for e in events if e.kind == "sentence"),
        }
        if extra:
            rec.update(extra)
        self.turns.append(rec)
        self._archive(rec)
        return rec

    def _archive(self, rec):
        """Append the turn to today's file. Never raises: a full disk must not stop the robot answering."""
        if not self.archive_dir or not (rec.get("text") or rec.get("reply")):
            return
        try:
            day = time.strftime("%Y-%m-%d", time.localtime(rec["at"]))
            os.makedirs(self.archive_dir, mode=0o700, exist_ok=True)   # guests' words: this user only
            row = {"robot": self.robot, **rec}
            with open(os.path.join(self.archive_dir, "conversations-%s.jsonl" % day), "a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")
            if self._pruned_on != day:
                self._pruned_on = day
                self._prune()
        except Exception as e:  # noqa: BLE001
            self.log("conversation log failed: %r" % (e,))

    def _prune(self):
        files = sorted(f for f in os.listdir(self.archive_dir) if f.startswith("conversations-") and f.endswith(".jsonl"))
        cutoff = self.clock() - self.keep_days * 86400
        total = 0
        for f in reversed(files):                                   # newest first
            p = os.path.join(self.archive_dir, f)
            total += os.path.getsize(p)
            if os.path.getmtime(p) < cutoff or total > self.max_mb * 1024 * 1024:
                os.remove(p)

    def set_health(self, name, ok, ms=None, note=""):
        self.health[name] = {"ok": ok, "ms": ms, "note": note, "at": self.clock()}

    def snapshot(self, n=30):
        turns = list(self.turns)[-n:]
        firsts = sorted(t["first_audio_s"] for t in turns if t["first_audio_s"] is not None)
        return {
            "turns": turns,
            "health": self.health,
            "mic": self.mic,
            "summary": {"count": len(turns),
                        "first_audio_median_s": firsts[len(firsts) // 2] if firsts else None,
                        "first_audio_worst_s": firsts[-1] if firsts else None},
        }
