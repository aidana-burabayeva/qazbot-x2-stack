#!/usr/bin/env python3
"""Adaptive model/effort router (SIM ONLY, stdlib only). Decides provider, model and reasoning effort from the task, not from quota alone.

Facts it relies on (checked on this host):
 * codex: model catalog + supported reasoning efforts are readable with `codex debug models` -> choices are validated against it.
 * claude: `--model <alias|full name>` and `--effort low|medium|high|xhigh|max` exist, but there is NO list of valid models. A Claude
   model is `verified` only after a real successful run (recorded by the orchestrator); until then it is used flagged `unverified`.
Principle: a strong model finds the cause and writes the reproducible test; a cheaper one implements the confirmed fix; a failed
regression gate escalates (effort first, then model) instead of repeating the same attempt forever.
"""
import json
import os
import re
import subprocess
import time

HIGH_TYPES = {'root_cause_analysis', 'tf', 'nav2', 'slam', 'mc_dynamics', 'localization', 'architecture', 'safety', 'collision'}
MEDIUM_TYPES = {'unit_tests', 'detailed_implementation', 'local_fix', 'automation', 'tooling'}
LOW_TYPES = {'log_classification', 'metric_extraction', 'documentation', 'structured_report'}
RISKY = {'safety', 'high'}

EFFORTS = ['low', 'medium', 'high', 'xhigh', 'max']
# preference lists (agent, model, effort); the first available one wins. Overridable by NIGHTLY_ROUTER_FILE (json).
PREFS = {
    'LOW': [('claude', 'claude-haiku-5-5', 'low'), ('codex', 'gpt-6.1-sol', 'low')],
    'MEDIUM': [('codex', 'gpt-6.1-sol', 'medium'), ('claude', 'claude-sonnet-5-5', 'medium')],
    'HIGH': [('claude', 'claude-opus-5-5', 'high'), ('codex', 'gpt-6.1-sol', 'high')],
}
CLAUDE_LADDER = ['claude-haiku-5-5', 'claude-sonnet-5-5', 'claude-opus-5-5']


def classify(task_type=None, complexity=None, risk=None):
    t, c, r = (task_type or '').lower(), (complexity or '').lower(), (risk or '').lower()
    if r in RISKY or t in HIGH_TYPES:
        return 'HIGH'
    if c == 'high':
        return 'HIGH' if r == 'medium' else 'MEDIUM'
    if t in LOW_TYPES and c in ('', 'low') and r in ('', 'low'):
        return 'LOW'
    return 'MEDIUM'  # unknown or mixed: do not guess cheap


def load_prefs():
    f = os.environ.get('NIGHTLY_ROUTER_FILE')
    if f and os.path.exists(f):
        d = json.load(open(f))
        return {k: [tuple(x) for x in v] for k, v in d.items()}
    return PREFS


def codex_catalog():
    """{slug: [supported efforts]} from the installed CLI (or a test seam)."""
    cmd = os.environ.get('NIGHTLY_CODEX_MODELS_CMD')
    try:
        raw = subprocess.run([cmd] if cmd else [os.environ.get('NIGHTLY_CODEX_BIN', 'codex'), 'debug', 'models'], capture_output=True, text=True, timeout=30).stdout
        d = json.loads(raw)
        ms = d.get('models', d) if isinstance(d, dict) else d
        return {m.get('slug') or m.get('id'): [e.get('effort') if isinstance(e, dict) else e for e in m.get('supported_reasoning_levels', [])] for m in ms}
    except Exception:
        return None  # catalog unreadable: unknown, not "everything supported"


def fit_effort(effort, supported):
    """Highest supported effort not above the requested one (never silently raises cost)."""
    if effort in supported:
        return effort
    want = EFFORTS.index(effort) if effort in EFFORTS else 0
    below = [e for e in supported if e in EFFORTS and EFFORTS.index(e) <= want]
    return max(below, key=EFFORTS.index) if below else None


def route(task_type=None, complexity=None, risk=None, escalation=0, catalog=None, claude_verified=None, allowed_agents=None):
    """Returns a list of candidate dicts in preference order: agent, model, effort, class, verified, reason."""
    cls = classify(task_type, complexity, risk)
    cat = catalog if catalog is not None else codex_catalog()
    cv = claude_verified or {}
    out = []
    for agent, model, effort in load_prefs()[cls]:
        if allowed_agents and agent not in allowed_agents:
            continue
        m, e = model, effort
        # escalation: first raise effort one step per level, then move up the model ladder (claude)
        for _ in range(escalation):
            if EFFORTS.index(e) < EFFORTS.index('xhigh'):
                e = EFFORTS[EFFORTS.index(e) + 1]
            elif agent == 'claude' and m in CLAUDE_LADDER and CLAUDE_LADDER.index(m) < len(CLAUDE_LADDER) - 1:
                m = CLAUDE_LADDER[CLAUDE_LADDER.index(m) + 1]
            elif e != 'max':
                e = 'max'
        if agent == 'codex':
            if cat is None:
                out.append(dict(agent=agent, model=m, effort=e, cls=cls, verified=False, reason='codex catalog unreadable'))
                continue
            if m not in cat:
                continue
            fe = fit_effort(e, cat[m])
            if fe is None:
                continue
            out.append(dict(agent=agent, model=m, effort=fe, cls=cls, verified=True, reason=f'catalog ok{"" if fe == e else f" (effort {e}->{fe})"}'))
        else:
            ok = e in (cv.get(m) or [])
            out.append(dict(agent=agent, model=m, effort=e, cls=cls, verified=ok, reason='verified by a real run' if ok else 'unverified: no model list in the claude CLI'))
    return out
