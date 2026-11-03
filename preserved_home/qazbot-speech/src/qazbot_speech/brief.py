"""The brief file: one text file built by the operator site (Context + Q&A sheet). Re-read when it changes.

Contract with x2-site/dance-bind (docs/adr/0002): the path is QBOT_VENUE_BRIEF_FILE there; here it is
`BRIEF_FILE`. The service never writes it.
"""
import os


class BriefFile:
    def __init__(self, path):
        self.path = path
        self._mtime = None
        self._text = ""

    @property
    def text(self):
        """Current brief; empty if the file does not exist yet. Cheap: one stat per call."""
        try:
            st = os.stat(self.path)
            m = (st.st_mtime_ns, st.st_size)   # size too: two writes inside one clock tick keep the same mtime
        except OSError:
            self._mtime, self._text = None, ""
            return ""
        if m != self._mtime:
            with open(self.path, encoding="utf-8") as f:
                self._text, self._mtime = f.read(), m
        return self._text
