"""OpenRouter streaming LLM with a hedged second request.

The first route starts at once. If it has produced no token after `hedge_sec`, the second route starts
too; whichever yields a token first wins and the other is cancelled. Tail latency drops to about
hedge_sec + the faster route's first token.
"""
import asyncio
import json

import aiohttp

URL = "https://openrouter.ai/api/v1/chat/completions"

# Measured on a laptop 2026-10-14 (docs/bench-2026-10-14-laptop.txt): fastest median, stable backup.
DEFAULT_ROUTES = (
    {"model": "openai/gpt-oss-120b", "provider": {"sort": "latency"},
     "reasoning": {"effort": "low", "exclude": True}},
    {"model": "google/gemini-2.5-flash-lite"},
)

RULES = {
    "kk": "Respond in Kazakh.",
    "ru": "Respond in Russian.",
    "en": "Respond in English.",
}
STYLE = ("You are QazBot, a friendly humanoid robot. Plain spoken text only: no emoji, no markdown, no lists. "
         "Usually answer in one or two short sentences; use up to three only when the question needs it.")


GENERAL = ("Answer general-knowledge questions (people, countries, facts, weather in general terms, jokes, songs, "
           "small talk) directly and briefly from your own knowledge, even when the venue notes do not mention "
           "them. Rules in the venue notes such as 'only say what is in this text' or 'if you do not know, send "
           "the guest to the desk' apply ONLY to questions about this venue or event that the notes do not cover. "
           "You stand in a busy place where people also talk to each other and about you. Answer only words "
           "addressed to you: a question, a request, a greeting, thanks, or a clear follow-up to your last answer. "
           "If the words are clearly not for you (people talking to each other, comments about you, the test or the "
           "demo, filler, half sentences, noise), output exactly <silent> and nothing else.")
SILENT = "<silent>"


def build_system(lang, brief):
    """Our style, the reply language, the operator's venue notes, and then OUR general-knowledge rule last, so a
    template brief ('refer anything unknown to the registration desk') cannot turn the robot into a
    desk-referral machine (seen on 2026-10-15: 'Кто президент Казахстана?' -> 'подойдите к стойке')."""
    parts = [STYLE, RULES.get(lang, "")]
    if brief and brief.strip():
        parts.append("Venue notes (about this place and event):" + chr(10) + brief.strip())
    parts.append(GENERAL)
    return (chr(10) * 2).join(p for p in parts if p)


class OpenRouterLLM:
    def __init__(self, api_key, routes=DEFAULT_ROUTES, hedge_sec=1.2, timeout=20, max_tokens=800):
        self.key, self.routes = api_key, routes
        self.hedge_sec, self.timeout, self.max_tokens = hedge_sec, timeout, max_tokens
        self._session = None

    def _http(self):
        if self._session is None or self._session.closed:  # keep-alive: no TLS setup per turn
            self._session = aiohttp.ClientSession(headers={"Authorization": "Bearer " + self.key})
        return self._session

    async def warm(self):
        """Open the connection ahead of the first turn."""
        try:
            async with self._http().get("https://openrouter.ai/api/v1/models?limit=1") as r:
                await r.read()
        except aiohttp.ClientError:
            pass

    async def _one(self, route, messages, out, tag):
        body = dict(route, stream=True, max_tokens=self.max_tokens, temperature=0.4, messages=messages)
        try:
            async with self._http().post(URL, json=body, timeout=aiohttp.ClientTimeout(total=self.timeout)) as r:
                if r.status != 200:
                    await out.put((tag, "error", "HTTP %d" % r.status))
                    return
                async for raw in r.content:
                    line = raw.strip()
                    if not line.startswith(b"data: ") or line == b"data: [DONE]":
                        continue
                    try:
                        piece = json.loads(line[6:])["choices"][0]["delta"].get("content")
                    except (ValueError, KeyError, IndexError):
                        continue
                    if piece:
                        await out.put((tag, "piece", piece))
            await out.put((tag, "done", None))
        except asyncio.CancelledError:
            raise
        except Exception as e:  # network blocked, reset, timeout
            await out.put((tag, "error", repr(e)))

    async def stream(self, system, history, user, lang):
        messages = [{"role": "system", "content": build_system(lang, system)}]
        messages += history + [{"role": "user", "content": user}]
        out, tasks, winner, failed = asyncio.Queue(), {}, None, set()

        def start(i):
            tasks[i] = asyncio.ensure_future(self._one(self.routes[i], messages, out, i))

        start(0)
        nxt = 1
        try:
            while True:
                wait = self.hedge_sec if (winner is None and nxt < len(self.routes)) else None
                try:
                    tag, kind, val = await asyncio.wait_for(out.get(), wait)
                except asyncio.TimeoutError:
                    start(nxt)
                    nxt += 1
                    continue
                if winner is not None and tag != winner:
                    continue
                if kind == "piece":
                    if winner is None:
                        winner = tag
                        for i, t in tasks.items():
                            if i != tag:
                                t.cancel()
                    yield val
                elif kind == "done":
                    return
                else:  # error
                    failed.add(tag)
                    if winner == tag:
                        raise RuntimeError("LLM stream failed mid-reply: %s" % val)
                    if nxt < len(self.routes):
                        start(nxt)
                        nxt += 1
                    elif len(failed) >= len(tasks):
                        raise RuntimeError("all LLM routes failed: %s" % val)
        finally:
            for t in tasks.values():
                t.cancel()
