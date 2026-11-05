# Smac clearance regression (plan-only stack)

Cause fixed: the costmap marks a cell inscribed by the distance between cell
*centres*, but the occupied area is the whole cell, so a path on cell centres is
up to half a cell diagonal (res/sqrt2 = 0.0354 m) closer to a wall than the radius asks.
`nav_plan.launch.py` adds that margin to the inscribed radius (`cell_margin:=true`, derived
from the map resolution, not tuned). `R = 0.65 m` is still a candidate envelope, not an approved footprint.

`harness.py` plans 20 random pairs (endpoints with exact clearance >= R + res/sqrt2, seed 7) and
measures the exact minimum distance from the dense path to the nearest non-free cell *square* on a
5 mm raster. It also runs a plan-only NavigateToPose.

| run | paths | violations of R | min clearance |
|---|---|---|---|
| PC, cell_margin=false (baseline) | 19 | 4 | 0.6255 m |
| PC, cell_margin=true | 17 | 0 | 0.6612 m |
| X2 (aarch64, Nav2 1.1.18), cell_margin=true | 17 | 0 | 0.6612 m |

The 3 paths lost with the fix have no route with centre-line clearance >= R at all.
Known limit: a robot that starts closer than R + res/sqrt2 to any mapped obstacle or unknown cell
cannot get a plan ("Starting point in lethal space"); stray single occupied cells in free space of
the D1 map trigger this.
