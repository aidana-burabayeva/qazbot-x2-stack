import pytest


@pytest.fixture(autouse=True)
def never_call_a_real_agent(monkeypatch):
    """Nightly tests must not spend provider quota or touch the real Codex session: real binaries, app-server socket and queue DB
    are unreachable unless a test installs a stub."""
    monkeypatch.setenv('NIGHTLY_CLAUDE_BIN', '/nonexistent/claude')
    monkeypatch.setenv('NIGHTLY_CODEX_BIN', '/nonexistent/codex')
    monkeypatch.setenv('NIGHTLY_CODEX_SOCK', '/nonexistent/app-server.sock')
    monkeypatch.setenv('NIGHTLY_CODEX_QUEUE_DB', '/nonexistent/queue.sqlite')
