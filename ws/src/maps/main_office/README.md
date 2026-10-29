# main_office

Edited D1 map supplied by the operator, with the unknown mask restored from the aligned original D1 export. The source editor export contained only 0/254 and had lost all unknown pixels. The derived map preserves all edited obstacles and changes only formerly unknown white pixels to 205.

Nav2 trinary semantics: 13712 occupied, 413052 unknown, 180955 free cells; `free_thresh: 0.196`. The actual Nav2 map_server classified zero unknown pixels as free. See `manifest.json` for hashes and source provenance, and `docs/P20_TWIN02_HANDOFF.md` for simulation evidence.

Both stack and simulator must use this same directory. The simulated world covers occupied cells exactly with physical walls; unknown is not invented solid geometry and does not become known free in the navigation map. No physical navigation qualification is claimed.
