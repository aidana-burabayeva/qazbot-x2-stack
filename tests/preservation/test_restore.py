"""Real offline restore drill, not an assertion against a historical PASS file.

Expected partial BLOCKED findings are explicit; readable source/archive checks
still fail if their current bytes or restored behavior regress.
"""
import importlib.util
import json
import os
from pathlib import Path
import tempfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("preservation", ROOT / "tools/verify_preservation_restore.py")
preservation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preservation)


@pytest.fixture(scope="module")
def drill():
    scratch = ROOT / ".qazbot/preservation_restore"
    scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pytest-", dir=scratch) as directory:
        report = preservation.verify(preservation.SOURCE, Path(directory))
        output = os.environ.get("QAZBOT_RESTORE_EVIDENCE")
        if output:
            Path(output).write_text(json.dumps(report, indent=2) + "\n")
        yield report


@pytest.mark.parametrize("name", ["robot_code_runtime_private.tar.gz", "historical_preservation.tar",
                                 "robot_git.bundle", "robot_git_preserved.bundle"])
def test_existing_archive_sha256(drill, name):
    archive = drill["archives"][name]
    assert archive["status"] == "OFFLINE_VERIFIED", archive
    assert archive["actual_sha256"] == archive["expected_sha256"]


def test_full_tar_stream_regular_permissions_and_links(drill):
    archive = drill["archives"]["robot_code_runtime_private.tar.gz"]
    assert archive["counts"]["inventory_regular_checked"] == 18431
    assert archive["counts"]["inventory_link_targets_checked"] == 268
    assert archive["counts"]["scratch_symlinks_recreated"] == 354
    assert not archive["mismatches"]
    assert not archive["missing_eligible_inventory"]
    assert not archive["unsafe_members"]
    assert len(archive["restored_samples"]) == 4
    assert all(x["matched"] for x in archive["restored_samples"])


def test_bundle_restores_manifested_current_production(drill):
    restore = drill["bundle_restore"]
    assert restore["status"] == "OFFLINE_VERIFIED", restore
    assert restore["sha"] == preservation.PRESERVED
    assert restore["pinned_commits"][preservation.HARDWARE] == preservation.HARDWARE
    assert restore["pinned_commits"][preservation.FEEDBACK] == preservation.FEEDBACK
    assert restore["checked_count"] == 374
    full = restore["full_source_test_tooling"]
    assert full["checked_count"] == 1974
    assert len(full["excluded_runtime_locks"]) == 73
    assert all(p.endswith(".lock") for p in full["excluded_runtime_locks"])
    assert not full["mismatch"] + full["unexpected_absent"]
    assert not restore["absent"] + restore["mismatch"] + restore["executable_mismatch"]
    assert restore["fsck"]["returncode"] == 0


def test_restored_d1_map_decodes_and_preserves_unknown(drill):
    map_ = drill["d1_map"]
    assert map_["status"] == "OFFLINE_VERIFIED"
    assert map_["size"] == [673, 903]
    assert map_["unknown_cells_trinary"] == 413052
    assert map_["pgm_sha256"] == "72ee4094b394241e7b2761878774efbdd99e93731d7cb52709b2231c3975b354"
    assert map_["yaml_sha256"] == "c86aa2aaefbefcaa2ddee994572cf6d13dd777b37b8e070de947f7685be43c1b"


def test_actual_backend_imports_and_separate_lab_scope(drill):
    imports = drill["backend_imports"]
    assert imports["process"]["returncode"] == 0
    assert imports["current_lab_process"]["returncode"] == 0
    for name in ("loc_trust", "control_geometry", "native_odom", "level_bridge",
                 "native_odom_bridge", "goal_coordinator", "x2_motion_adapter",
                 "costmap_emul", "replay_d04", "cm_eval_lib", "rclpy", "rosbag2_py", "sensor_msgs_py"):
        assert imports["modules"][name]["status"] == "OFFLINE_VERIFIED", imports["modules"][name]
    for name in ("source_state", "lab", "labctl", "preflight"):
        assert imports["current_lab_modules"][name]["status"] == "OFFLINE_VERIFIED", imports["current_lab_modules"][name]
    assert imports["modules"]["aimdk_msgs"]["status"] == "BLOCKED"
    # Importing adapter's pure core does not exercise its delayed SDK import.
    assert "aimdk_msgs" in imports["modules"]["aimdk_msgs"]["error"]


def test_isolated_real_dependency_projector_build(drill):
    build = drill["projector"]
    assert build["status"] == "OFFLINE_VERIFIED", build
    assert build["configure"]["returncode"] == build["build"]["returncode"] == build["install"]["returncode"] == 0
    assert "-DBUILD_TESTING=OFF" in build["configure"]["cmd"]


def test_all_three_bags_have_explicit_copy_and_hash_disposition(drill):
    assert len(drill["bags"]) == 3
    assert sum(x["expected_size"] for x in drill["bags"]) == 16438766865
    for bag in drill["bags"]:
        assert len(bag["expected_sha256"]) == 64
        assert bag["checks"]
        verified = {(c["device"], c["inode"]) for c in bag["checks"]
                    if c.get("sha256") == bag["expected_sha256"] and c.get("size") == bag["expected_size"]
                    and c.get("metadata", {}).get("readable")}
        assert bag["independent_file_copies"] == len(verified)
        assert bag["status"] == ("OFFLINE_VERIFIED" if len(verified) >= 2 else "BLOCKED")
        if bag["status"] == "BLOCKED":
            assert "independent_copies:" + bag["name"] in drill["blocked"]


def test_missing_runtime_targets_and_partial_backup_cannot_be_full_restore_pass(drill):
    deps = drill["dependencies"]
    assert len(deps["inventory_errors"]) == 86
    assert deps["missing_archive_link_targets"]
    assert all(x["resolution"] != "archived" for x in deps["missing_archive_link_targets"])
    assert deps["historical_partial"][0]["size"] == 1142755328
    assert "incomplete_historical_backup" in drill["blocked"]
    assert drill["status"] == "RESTORE_BLOCKED"
    assert drill["classification"] == "OFFLINE_VERIFIED"
    assert drill["d05_independent_work"].startswith("UNBLOCKED")
