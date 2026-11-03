"""Provider interfaces. Real providers (Soniox, OpenRouter, Mangisoz) and test fakes implement these."""
from typing import AsyncIterator, Optional, Tuple

try:
    from typing import Protocol
except ImportError:  # Python 3.7 on old robot images
    Protocol = object


class STT(Protocol):
    async def transcribe(self, audio: bytes) -> Tuple[str, Optional[str]]:
        """Return (text, language id or None)."""


class LLM(Protocol):
    def stream(self, system: str, history: list, user: str, lang: str) -> AsyncIterator[str]:
        """Yield reply text pieces as they arrive."""


class TTS(Protocol):
    def synth(self, text: str, lang: str) -> AsyncIterator[bytes]:
        """Yield audio chunks for `text`."""


class Speaker(Protocol):
    async def play(self, chunk: bytes) -> None: ...

    async def gesture(self, motion: int) -> None: ...
