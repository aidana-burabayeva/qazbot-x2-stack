"""Context tools: facts fetched for one turn and handed to the LLM next to the brief.

The model cannot know the weather, and guessed a city from the brief. `Weather` fetches the current weather
for the venue city (Open-Meteo, no key) when the guest asks about it, and the model words the answer in the
guest's language. Failure or a slow network gives no context, never a stuck turn.
"""
import asyncio
import json
import re
import time

import aiohttp

WEATHER_ASK = re.compile(r"погод|температур|градус|дожд|снег|жарко|холодно|ауа\s*райы|жаңбыр|қар\b|weather|temperature|"
                         r"rain|snow|how (hot|cold)|forecast|прогноз", re.I)
_CITY = re.compile(r"(?:г\.|город|қала|city)\s*[:\s]*([А-ЯЁӘІҢҒҮҰҚӨҺA-Z][\w\-]+)|Место:[^\n]*?,\s*(?:г\.\s*)?([А-ЯЁA-Z][\w\-]+)", re.I)
_CODES = {0: "ясно", 1: "в основном ясно", 2: "переменная облачность", 3: "пасмурно", 45: "туман", 48: "туман",
          51: "морось", 53: "морось", 55: "морось", 61: "небольшой дождь", 63: "дождь", 65: "сильный дождь",
          71: "небольшой снег", 73: "снег", 75: "сильный снег", 80: "ливень", 81: "ливень", 82: "сильный ливень",
          95: "гроза", 96: "гроза с градом", 99: "гроза с градом"}


def venue_city(brief):
    m = _CITY.search(brief or "")
    return (m.group(1) or m.group(2)) if m else None


class Weather:
    def __init__(self, timeout=3.5, ttl=600.0, clock=time.monotonic, fetch=None):
        self.timeout, self.ttl, self.clock = timeout, ttl, clock
        self._fetch = fetch or self._open_meteo
        self._cache = {}

    def matches(self, text):
        return bool(WEATHER_ASK.search(text or ""))

    async def context(self, text, brief):
        if not self.matches(text):
            return None
        city = venue_city(brief)
        if not city:
            return None
        hit = self._cache.get(city)
        if hit and self.clock() - hit[0] < self.ttl:
            return hit[1]
        line = None
        for _ in range(2):          # the free API sometimes answers with an error body once
            try:
                line = await asyncio.wait_for(self._fetch(city), self.timeout)
                break
            except Exception:
                continue
        if line:
            self._cache[city] = (self.clock(), line)
        return line

    async def _open_meteo(self, city):
        async with aiohttp.ClientSession() as s:
            async with s.get("https://geocoding-api.open-meteo.com/v1/search",
                             params={"name": city, "count": 1, "language": "ru"}) as r:
                g = json.loads(await r.text()).get("results")
            if not g:
                return None
            async with s.get("https://api.open-meteo.com/v1/forecast", params={
                    "latitude": g[0]["latitude"], "longitude": g[0]["longitude"], "timezone": "auto",
                    "current": "temperature_2m,apparent_temperature,weather_code,wind_speed_10m",
                    "daily": "temperature_2m_max,temperature_2m_min"}) as r:
                d = json.loads(await r.text())
        c, day = d["current"], d["daily"]
        return ("Погода сейчас в городе %s (источник Open-Meteo, данные свежие): %s, %d°C, ощущается как %d°C, ветер %d км/ч; "
                "сегодня от %d до %d°C. Если гость спросил о погоде, ответь по этим данным." % (
                    g[0]["name"], _CODES.get(c["weather_code"], "без осадков"), round(c["temperature_2m"]),
                    round(c["apparent_temperature"]), round(c["wind_speed_10m"]),
                    round(day["temperature_2m_min"][0]), round(day["temperature_2m_max"][0])))
