#!/usr/bin/env python3
"""Send one key to the MuJoCo window via XTEST (the official sim has no documented ROS reset command).
Usage: gui_key.py KEYSYM   e.g. BackSpace (MuJoCo: reset simulation), space (pause/run). Host-side, display only."""
import sys, time
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / 'runtime_x86/run/pylib'))   # pip install --target runtime_x86/run/pylib python-xlib
from Xlib import X, XK, display
from Xlib.ext import xtest
d = display.Display()
def find(w):
    name = w.get_wm_name()
    if name and 'MuJoCo' in str(name):
        return w
    for c in w.query_tree().children:
        r = find(c)
        if r: return r
win = find(d.screen().root)
if not win: raise SystemExit('MuJoCo window not found')
geo = win.get_geometry(); tr = win.translate_coords(d.screen().root, 0, 0)
if sys.argv[1] == 'click':           # gui_key.py click X Y : window-relative pixel click (MuJoCo UI buttons are drawn, not widgets)
    d.screen().root.warp_pointer(-tr.x + int(sys.argv[2]), -tr.y + int(sys.argv[3])); d.sync(); time.sleep(0.3)
    xtest.fake_input(d, X.ButtonPress, 1); d.sync(); time.sleep(0.1); xtest.fake_input(d, X.ButtonRelease, 1); d.sync()
    print('clicked', sys.argv[2:4], 'in', win.get_wm_name()); raise SystemExit
# click an inert point of the 3D view so the window has focus (right panel, empty area), then send the key
cx, cy = -tr.x + geo.width * 0.62, -tr.y + geo.height * 0.6
d.screen().root.warp_pointer(int(cx), int(cy)); d.sync(); time.sleep(0.2)
xtest.fake_input(d, X.ButtonPress, 1); xtest.fake_input(d, X.ButtonRelease, 1); d.sync(); time.sleep(0.2)
k = d.keysym_to_keycode(XK.string_to_keysym(sys.argv[1]))
xtest.fake_input(d, X.KeyPress, k); d.sync(); time.sleep(0.05); xtest.fake_input(d, X.KeyRelease, k); d.sync()
print('sent', sys.argv[1], 'to', win.get_wm_name())
