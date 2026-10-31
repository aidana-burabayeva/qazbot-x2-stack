# Navigation frame fix: independent review

Review date: 2026-10-31. Environment: local attempt worktree, offline Python tests only; no robot, network, ROS node or simulator contacted/started. Reviewed committed diff `fdd692a7d2c355c8abc22d5e446ec6b1b6d6fe37..ba40df5c8772512caf0cf334626c93ae36867eb3` (`git diff HEAD^ HEAD`). Initial worktree status was clean. The diff contains only geometry.py, plant/hal_plant_node.py and test_frame_anchor.py under digital_twin/ws/src/x2_twin_sim. Existing HANDOFF/TEST_STATUS and measured frame_root_cause README/probe_output were read; their measurements are historical evidence, not new runs.

## Consumers

Inventory commands: `git grep -n -E 'FrameAligner|HalPlantNode|hal_plant'` across tracked repository files, plus `rg -n 'initialised|_origin_inv' digital_twin tests tools`. Paths below are repository-relative. No additional runtime constructor, subclass or direct caller was found.

| Consumer/reference | Behaviour and explicit anchor requirement |
| --- | --- |
| digital_twin/ws/src/x2_twin_sim/x2_twin_sim/geometry.py | Definition, not another consumer. Class default remains first_sample; nominal is opt-in. |
| digital_twin/ws/src/x2_twin_sim/x2_twin_sim/plant/hal_plant_node.py | Sole runtime FrameAligner constructor. HalPlantNode now declares ROS parameter anchor with default nominal and passes it explicitly; main calls spin(HalPlantNode). Intentional frame change. |
| digital_twin/ws/src/x2_twin_sim/test/test_geometry.py | Existing default constructor remains first_sample; unchanged behaviour and source. No explicit parameter required. |
| digital_twin/ws/src/x2_twin_sim/test/test_frame_anchor.py | Reproduction explicitly retains first_sample; nominal test explicitly selects nominal and loses its strict xfail. Both now pass. |
| tests/nightly/test_stall_frame_audit.py | Two default constructors retain legacy behaviour. These audit tests describe the historical mismatch; they do not demonstrate that the fixed HAL node still has it. No explicit parameter required. |
| digital_twin/ws/src/x2_twin_sim/setup.py | Console entry hal_plant resolves to hal_plant_node:main. Same entry point, now gets corrected node default. No explicit anchor required. |
| digital_twin/ws/src/x2_twin_bringup/launch/twin.launch.py | Sole launch creation of hal_plant, only for plant:=official. Passes twin.yaml and map/start/scan overrides without anchor, so corrected nominal default applies. No launch edit required. Lite branch uses kinematic_plant, unaffected. |
| digital_twin/ws/src/x2_twin_bringup/config/twin.yaml | hal_plant parameter section has no anchor override; nominal default applies. No edit required for corrected behaviour. |
| digital_twin/docker/entrypoints/stack_health.sh | Checks node name only; name unchanged, health-check behaviour unchanged. No parameter needed. |
| digital_twin/ws/src/x2_twin_sim/x2_twin_sim/loc/truth_localizer_node.py | FrameAligner mention is documentation, no import/construction. Indirectly consumes plant TF contract: identity map->odom remains valid with corrected map-coordinate odom. No code/parameter change required. |
| digital_twin/README.md | Architecture/tree mentions only; no executable consumer or behaviour change. |
| docs/nightly/CODEX_STALL_AB.md; docs/evidence/nightly_20261030/frame_root_cause/README.md | Historical first-sample explanation, unchanged evidence; no executable consumer. |
| docs/nightly/loops/nav_frame.json | Task specifications mentioning symbols; no runtime plant consumer. No parameter change needed. |
| docs/evidence/nightly_20261030/codex_stall_ab/{A,B}/stack.log; docs/evidence/nightly_20261030/l2_doorway_stall/stack.log; docs/evidence/nightly_20261030/l2_stall_s1_0046/stack.log; docs/evidence/p20_twin02_20261029/nav2_stack.log | Historical hal_plant process/log references only. Unchanged evidence, no parameter requirement. |

Indirect runtime path: HalPlantNode.current_state feeds PlantNode in digital_twin/ws/src/x2_twin_sim/x2_twin_sim/plant/base.py. Its odom, odom->base_footprint TF, model_truth and map raycast scans therefore all use the corrected pose. Downstream truth-localization or AMCL/Nav2 receives corrected inputs; their algorithms/configuration are unchanged. This is an intended observable input change, not a claim that goal outcomes remain identical.

## Behaviour change

Previously HAL plant used `start ⊕ inverse(first_HAL) ⊕ current_HAL`. It now defaults to `start ⊕ current_HAL`, matching the inverse of world.map_to_sim and the nominal identity spawn. A late settled first sample retains its measured displacement/yaw rather than redefining the navigation frame. There is no empirical offset or wall relocation. The test covers translated and rotated starts, settled-first sample order, identity and multiple subsequent poses.

FrameAligner(start) still anchors on the first sample. HalPlantNode is the intentional default switch. Neither twin.launch.py nor twin.yaml must explicitly set anchor for the fix to apply. To reproduce legacy HAL behaviour, explicitly set the ROS node parameter `hal_plant.ros__parameters.anchor: first_sample` in its YAML, or use `--ros-args -p anchor:=first_sample` when running the executable directly. The launch has no declared anchor argument: `anchor:=first_sample` is not an exposed launch override. An explicit nominal YAML setting is optional, not required.

A constant rigid frame correction leaves body-forward finite-difference speed and yaw-rate mathematically invariant (up to floating-point roundoff); map pose, heading and raycast results change consistently. Topic names, rates, frames and safety settings are unchanged.

## Risks

- Contract assumes HAL poses are in the world frame with nominal spawn (0,0,0), as in the supplied measured probe. A different spawn/world-origin contract needs a deliberate design change; first_sample would reintroduce this mismatch against fixed walls. The unrelated collapse/respawn in the historical probe is not fixed here.
- Dataclass positional API changes: the second positional argument used to be _origin_inv and is now anchor. No repository caller supplies that positional argument or custom _origin_inv. External callers, if any, require review; none are assumed here.
- nominal makes initialised true immediately because the origin is known, before receiving a sample. No repository consumer reads this property. HAL pose still starts None and publishes no state before a sample.
- Invalid anchor values raise ValueError during construction rather than silently choosing a frame. The ordinary launch/config supplies no invalid override.
- The offline suite validates geometry and world agreement, but does not instantiate HalPlantNode or prove ROS parameter delivery, fresh simulator E2E navigation, or physical acceptance. Nominal-invalid-mode/property edge cases have no dedicated added tests. Historical audit results must not be relabelled as post-fix E2E results.

## Verdict

PASS for the scoped source review: no blocking repository regression found. The fix is minimal, preserves default library callers, and corrects the official HAL plant frame contract without launch edits. P20, localization code, controller, costmap, tolerances, footprint and safety logic are untouched. Independent read-only second-agent inspection agreed on consumers, launch defaults and the positional-API caveat.

Executed locally on 2026-10-31: `python3 -m pytest -q digital_twin/ws/src/x2_twin_sim/test` — **31 passed, 2 skipped in 0.83s** (exit 0). Suite contains environment skips for geometry_msgs availability and the unmounted I1 parity candidate. This verdict approves the frame-anchor implementation within offline review scope; runtime E2E remains unverified here.
