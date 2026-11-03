"""Local API and log page for the speech service (default 127.0.0.2:18100).

Read side feeds the robot-info site. Safe buttons only: test a phrase, clear memory, rerun the self-test.
Knowledge uploads are NOT here: they live on the operator site (docs/adr/0002).
"""
import json

from aiohttp import web

PAGE = """<!doctype html><meta charset="utf-8"><title>qazbot-speech</title>
<style>body{font:14px system-ui;margin:16px;max-width:980px}table{border-collapse:collapse;width:100%}
td,th{border-bottom:1px solid #8884;padding:4px 8px;text-align:left}.bad{color:#c33}.ok{color:#2a7}</style>
<h1>qazbot-speech</h1><div id=s></div><h2>Provider health</h2><div id=h></div>
<h2>Recent turns</h2><table id=t></table>
<h2>Log</h2><pre id=l style="max-height:320px;overflow:auto;font:12px monospace"></pre>
<p><button onclick="post('/api/clear-memory')">Clear memory</button>
<button onclick="post('/api/self-test')">Run self-test</button></p>
<script>
function post(u){fetch(u,{method:'POST'}).then(load)}
function esc(x){return String(x==null?'':x).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function load(){fetch('/api/status').then(r=>r.json()).then(d=>{
 var m=d.summary;document.getElementById('s').textContent='mic: '+(d.mic.active||'?')+' | turns: '+m.count+
  ' | first audio median '+m.first_audio_median_s+' s, worst '+m.first_audio_worst_s+' s';
 document.getElementById('h').innerHTML=Object.keys(d.health).map(k=>'<span class='+(d.health[k].ok?'ok':'bad')+'>'+esc(k)+
  ' '+(d.health[k].ok?'ok':'FAIL')+' '+esc(d.health[k].ms||'')+' ms '+esc(d.health[k].note)+'</span>').join(' &nbsp; ');
 document.getElementById('t').innerHTML='<tr><th>text<th>lang<th>stt<th>1st sentence<th>1st audio<th>reply</tr>'+
  d.turns.slice().reverse().map(t=>'<tr><td>'+esc(t.text)+'<td>'+esc(t.lang)+'<td>'+esc(t.stt_s)+'<td>'+esc(t.first_sentence_s)+
  '<td>'+esc(t.first_audio_s)+(t.fast_path?' (fast)':'')+(t.bridge?' (bridge)':'')+'<td>'+esc(t.reply)+'</tr>').join('')})}
function logs(){fetch('/api/log?n=150').then(r=>r.json()).then(d=>{var e=document.getElementById('l'),b=e.scrollTop+e.clientHeight>=e.scrollHeight-8;
 e.textContent=d.lines.map(x=>new Date(x.at*1000).toLocaleTimeString()+'  '+x.msg).join('\\n');if(b)e.scrollTop=e.scrollHeight})}
load();logs();setInterval(load,3000);setInterval(logs,3000)
</script>"""


DEFAULT_CONFIG = {"wake_names": [], "idle_sleep": 15}


def make_app(telemetry, on_clear_memory=None, on_self_test=None, on_say=None, config=None):
    async def status(_):
        return web.json_response(telemetry.snapshot(), dumps=lambda o: json.dumps(o, ensure_ascii=False))

    async def page(_):
        return web.Response(text=PAGE, content_type="text/html")

    async def clear(_):
        if on_clear_memory:
            on_clear_memory()
        return web.json_response({"ok": True})

    async def self_test(_):
        res = await on_self_test() if on_self_test else {}
        return web.json_response({"ok": True, "result": res})

    async def say(request):
        body = await request.json()
        text = str(body.get("text", ""))[:300]
        if not text or on_say is None:
            return web.json_response({"ok": False}, status=400)
        await on_say(text, body.get("lang"))
        return web.json_response({"ok": True})

    async def get_log(request):
        try:
            n = max(1, min(400, int(request.query.get("n", 100))))
        except ValueError:
            n = 100
        lines = list(getattr(telemetry, "lines", []))[-n:]
        return web.json_response({"lines": lines}, dumps=lambda o: json.dumps(o, ensure_ascii=False))

    async def get_config(_):
        # echo the settings the service really uses: stored values over the defaults
        return web.json_response({**DEFAULT_CONFIG, **(config() if config else {})})

    app = web.Application()
    app.add_routes([web.get("/", page), web.get("/api/status", status), web.get("/api/config", get_config), web.get("/api/log", get_log),
                    web.post("/api/clear-memory", clear), web.post("/api/self-test", self_test),
                    web.post("/api/say", say)])
    return app


def serve(app, host="127.0.0.2", port=18100):
    """127.0.0.2 by default, never 127.0.0.1: Tailscale forwards tailnet connections to 127.0.0.1 (see __main__)."""
    web.run_app(app, host=host, port=port, print=None)
