import pytest


@pytest.fixture(autouse=True)
def never_call_a_real_agent(monkeypatch):
    """Nightly tests must not spend provider quota: real claude/codex binaries are unreachable unless a test installs a stub."""
    monkeypatch.setenv('NIGHTLY_CLAUDE_BIN', '/nonexistent/claude')
    monkeypatch.setenv('NIGHTLY_CODEX_BIN', '/nonexistent/codex')
