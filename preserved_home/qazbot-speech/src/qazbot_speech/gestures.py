"""Greeting -> arm motion. Kazakh greetings give a handshake (1003), other greetings and goodbyes a wave (1002).

Motion codes and the control areas each accepts were measured on an X2 Ultra (Digital Bridge, 2026-09-17,
gesture_policy.MOTION_AREAS): the same code is accepted on one area and refused on another. McControlArea is
a bit field: LEFT_HAND=1, RIGHT_HAND=2, HEAD=4, WAIST=8. The right hand is preferred.
The mapping is data (`GREETINGS`), meant to be overridden per deployment.
"""
import re

HANDSHAKE = 1003
WAVE = 1002
RAISE_HAND = 1001

MOTION_AREAS = {
    1001: {1, 2},          # raise hand
    1002: {1, 2, 3},       # wave
    1003: {1, 2},          # handshake
    1008: {1, 2, 3},       # high five
    3001: {11},            # bow
}
PREFERRED_AREAS = (2, 1, 3, 11)   # right hand first

# Order matters: the first pattern that matches wins.
GREETINGS = (
    (HANDSHAKE, re.compile(r"ассалаум|ассалам|assalaum|салам\s*алейкум|уағалейкум|сәлеметсіз|салеметсиз|"
                           r"сәлем\b|салем\b|қол\s*алыс|кол\s*алыс|пожм\w*\s+(мне\s+)?рук|дай\s+руку|shake\s+(my\s+)?hand",
                           re.I)),
    (RAISE_HAND, re.compile(r"подними\s+(правую\s+|левую\s+|одну\s+)?руку|қолыңды\s+көтер|raise\s+(your\s+)?hand", re.I)),
    (WAVE, re.compile(r"(?<!\w)(привет|здравствуй\w*|добрый\s+(день|вечер)|доброе\s+утро|hello|hi|hey|"
                      r"пока|до\s+свидания|сау\s+бол|көріскенше|goodbye|bye)(?!\w)", re.I)),
)


def greeting_motion(text, greetings=GREETINGS):
    for code, pattern in greetings:
        if pattern.search(text):
            return code
    return None


def area_for(code):
    """The control area to send with `code`: the first preferred area the motion accepts."""
    accepted = MOTION_AREAS.get(code, {2})
    for a in PREFERRED_AREAS:
        if a in accepted:
            return a
    return min(accepted)
