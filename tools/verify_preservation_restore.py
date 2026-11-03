"""Offline restore drill against existing manifests; never starts a ROS participant.

Private content is hashed, not copied into evidence. All writes stay in the supplied
scratch directory. Missing data is reported separately from source restoration.
"""
import argparse
import collections
import datetime
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile

PRESERVED = "dd1d0af5bfb30733db33811a8cb0b3d8d632f13e"
FEEDBACK = "617999a77b3af19a5dc80175e16b52066ea3c7db"
HARDWARE = "1212d7eda003022d74c6ecc4a2c745fd48f57c2f"
SOURCE = Path("/home/adn/QazBot/x2_preservation_20261103")
ROOT = Path(__file__).resolve().parents[1]


def digest(stream):
    h = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        h.update(block)
    return h.hexdigest()


def file_hash(path):
    with path.open("rb") as stream:
        return digest(stream)


def run(args, cwd=None, env=None):
    p = subprocess.run(args, cwd=cwd, env=env, capture_output=True, text=True,
                       timeout=300)
    return {"cmd": args, "returncode": p.returncode,
            "stdout": p.stdout, "stderr": p.stderr}


def git(repo, *args):
    result = run(["git", "-C", str(repo), *args])
    if result["returncode"]:
        raise RuntimeError(result)
    return result["stdout"]


def production(path):
    parts = PurePosixPath(path).parts
    return (path.startswith(("ws/src/", "tools/", "tests/"))
            and not set(parts) & {"__pycache__", "build", "install", "log", "logs", "runtime", "results", "integration_results", "backups", "candidates"}
            and not any(x in parts[-1] for x in (".bak", "_candidate", "_rejected"))
            and not path.endswith((".pyc", ".evidence_copy")))


def archives(source, inventory, scratch):
    results = {}
    expected = {}
    for line in (source / "SHA256SUMS").read_text().splitlines():
        sha, name = line.split(maxsplit=1)
        expected[Path(name).name] = sha
    for name, sha in expected.items():
        try:
            actual = file_hash(source / name)
            results[name] = {"expected_sha256": sha, "actual_sha256": actual,
                             "status": "OFFLINE_VERIFIED" if sha == actual else "FAIL"}
        except OSError as exc:
            results[name] = {"status": "BLOCKED", "error": str(exc)}
    for name in ("robot_code_runtime_private.tar.gz", "historical_preservation.tar"):
        if results[name]["status"] != "OFFLINE_VERIFIED":
            continue
        info = results[name]
        counts = collections.Counter()
        mismatches, links, unknown, partial, unsafe = [], [], [], [], []
        seen = set()
        restored_samples = []
        link_scratch = scratch / name / "links"
        link_scratch.mkdir(parents=True)
        sample_names = {"control_backend/ws/src/lio/level_bridge.py",
                        "control_backend/ws/src/navigation/run_nav_full.sh",
                        "control_backend/ws/src/maps/d1_qazbot_20261029/map.pgm",
                        "control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml"}
        try:
            with tarfile.open(source / name, "r|*") as archive:
                for member in archive:
                    counts["members"] += 1
                    path = member.name.removeprefix("./")
                    if PurePosixPath(path).is_absolute() or ".." in PurePosixPath(path).parts:
                        unsafe.append(path)
                    row = inventory.get(path)
                    seen.add(path)
                    if member.isfile():
                        counts["regular"] += 1
                        actual = digest(archive.extractfile(member))
                        if path in sample_names:
                            # Stream mode cannot rewind: select small files for a
                            # second, bounded extraction pass below.
                            restored_samples.append(path)
                        if row and "symlink" not in row and "sha256" in row:
                            counts["inventory_regular_checked"] += 1
                            if (actual != row["sha256"] or member.size != row["size"]
                                    or member.mode & 0o777 != int(row["mode"], 8)):
                                mismatches.append(path)
                        elif name.startswith("robot_code"):
                            unknown.append(path)
                        if path.endswith(".partial"):
                            partial.append({"path": path, "size": member.size, "sha256": actual})
                    elif member.issym() or member.islnk():
                        counts["links"] += 1
                        # Inventory mode/hash follow the target, not lstat: compare
                        # the recorded link text only. Never resolve against this host.
                        if row and "symlink" in row:
                            counts["inventory_link_targets_checked"] += 1
                            if row["symlink"] != member.linkname:
                                mismatches.append(path)
                        links.append({"path": path, "target": member.linkname,
                                      "kind": "symlink" if member.issym() else "hardlink",
                                      "mode": oct(member.mode),
                                      "inventory_error": (row or {}).get("error")})
                        if member.issym():
                            probe = link_scratch / str(len(links))
                            probe.symlink_to(member.linkname)
                            if os.readlink(probe) != member.linkname or not stat.S_ISLNK(probe.lstat().st_mode):
                                mismatches.append(path)
                            counts["scratch_symlinks_recreated"] += 1
                    elif member.isdir():
                        counts["directories"] += 1
                    else:
                        unsafe.append(path)
            info.update(counts=counts, mismatches=mismatches, unmanifested_regular=unknown,
                        unsafe_members=unsafe, partial_files=partial, links=links)
            if name.startswith("robot_code"):
                eligible = {p for p, r in inventory.items() if "sha256" in r and "symlink" not in r
                            and not set(PurePosixPath(p).parts) & {"__pycache__", ".pytest_cache", ".git"}}
                info["missing_eligible_inventory"] = sorted(eligible - seen)
            if mismatches or unknown or unsafe or info.get("missing_eligible_inventory"):
                info["status"] = "FAIL"
            # Classify links using archive membership, without following private paths.
            for link in links:
                target = link["target"]
                if target.startswith("/home/agi/"):
                    target = target[len("/home/agi/"):]
                elif not target.startswith("/"):
                    target = os.path.normpath(str(PurePosixPath(link["path"]).parent / target))
                link["target_member_present"] = target in seen
            info["unresolved_link_count"] = sum(not x["target_member_present"] for x in links)
            # Resolve archive-internal link chains and directory aliases without
            # consulting /home/agi or /opt on this PC.
            by_path = {x["path"]: x for x in links}
            for link in links:
                current = link["path"]
                visited = set()
                while True:
                    if current in visited:
                        link["resolution"] = "cycle"
                        break
                    visited.add(current)
                    parts = PurePosixPath(current).parts
                    alias = next((str(PurePosixPath(*parts[:i])) for i in range(1, len(parts)+1)
                                  if str(PurePosixPath(*parts[:i])) in by_path), None)
                    if alias is None:
                        link["resolution"] = "archived" if current in seen else "missing_target"
                        break
                    item = by_path[alias]
                    target = item["target"]
                    if target.startswith("/home/agi/"):
                        target = target[len("/home/agi/"):]
                    elif not target.startswith("/") and item["kind"] == "symlink":
                        target = str(PurePosixPath(alias).parent / target)
                    remainder = parts[len(PurePosixPath(alias).parts):]
                    current = os.path.normpath(str(PurePosixPath(target).joinpath(*remainder)))
                link["resolved_target"] = current
            info["unresolved_link_chain_count"] = sum(x["resolution"] != "archived" for x in links)
            info["restored_samples"] = []
            if restored_samples:
                with tarfile.open(source / name, "r|*") as archive:
                    for member in archive:
                        if member.name not in restored_samples:
                            continue
                        target = scratch / name / "samples" / member.name
                        target.parent.mkdir(parents=True, exist_ok=True)
                        with archive.extractfile(member) as src, target.open("wb") as dst:
                            shutil.copyfileobj(src, dst)
                        target.chmod(member.mode & 0o777)
                        row = inventory[member.name]
                        matched = (file_hash(target) == row["sha256"]
                                   and target.stat().st_mode & 0o777 == int(row["mode"], 8))
                        info["restored_samples"].append({"path": member.name, "mode": oct(member.mode),
                                                         "sha256": file_hash(target), "matched": matched})
                        if not matched:
                            info["status"] = "FAIL"
        except (OSError, tarfile.TarError, EOFError) as exc:
            info.update(status="BLOCKED", error=str(exc))
    return results


def restore_bundle(source, inventory, scratch):
    repo = scratch / "restored"
    clone = run(["git", "clone", "--no-checkout", "--no-hardlinks",
                 str(source / "robot_git_preserved.bundle"), str(repo)])
    if clone["returncode"]:
        return {"status": "BLOCKED", "clone": clone}, None
    verify = run(["git", "-C", str(repo), "bundle", "verify", str(source / "robot_git_preserved.bundle")])
    original_verify = run(["git", "-C", str(repo), "bundle", "verify", str(source / "robot_git.bundle")])
    fsck = run(["git", "-C", str(repo), "fsck", "--full"])
    tree = git(repo, "ls-tree", "-rl", PRESERVED)
    total = sum(int(line.split("\t", 1)[0].split()[-1]) for line in tree.splitlines()
                if line.split("\t", 1)[0].split()[-1].isdigit())
    if shutil.disk_usage(scratch).free < total * 2 + 256 * 1024**2:
        return {"status": "BLOCKED", "reason": "insufficient space before checkout", "tree_bytes": total}, None
    git(repo, "checkout", "--detach", PRESERVED)
    checked, absent, mismatch, modes = [], [], [], []
    full_checked, full_absent, full_mismatch = [], [], []
    for path, row in inventory.items():
        if not path.startswith("control_backend/") or "sha256" not in row:
            continue
        rel = path[len("control_backend/"):]
        if rel.startswith(("ws/src/", "tools/", "tests/")) and "__pycache__" not in PurePosixPath(rel).parts and not rel.endswith(".pyc"):
            target = repo / rel
            if not target.is_file():
                full_absent.append(rel)
            elif file_hash(target) != row["sha256"]:
                full_mismatch.append(rel)
            else:
                full_checked.append(rel)
        if not production(rel):
            continue
        target = repo / rel
        if not target.is_file():
            absent.append(rel)
        elif file_hash(target) != row["sha256"]:
            mismatch.append(rel)
        else:
            checked.append(rel)
            # Git only restores executable bits, not group-write/ownership.
            if bool(target.stat().st_mode & stat.S_IXUSR) != bool(int(row["mode"], 8) & stat.S_IXUSR):
                modes.append(rel)
    pins = {sha: git(repo, "rev-parse", "--verify", sha + "^{commit}").strip()
            for sha in (PRESERVED, FEEDBACK, HARDWARE)}
    unexpected_absent = [p for p in full_absent if not p.endswith(".lock")]
    return {"status": "OFFLINE_VERIFIED" if not absent + mismatch + modes + unexpected_absent + full_mismatch and not verify["returncode"] and not original_verify["returncode"] and not fsck["returncode"] else "FAIL",
            "sha": git(repo, "rev-parse", "HEAD").strip(), "tree_bytes": total,
            "bundle_verify": verify, "original_bundle_verify": original_verify,
            "pinned_commits": pins, "fsck": fsck, "checked_count": len(checked),
            "checked": checked, "absent": absent, "mismatch": mismatch, "executable_mismatch": modes,
            "full_source_test_tooling": {"checked_count": len(full_checked), "checked": full_checked,
                                        "mismatch": full_mismatch, "excluded_runtime_locks": full_absent,
                                        "unexpected_absent": unexpected_absent},
            "refs": git(repo, "show-ref"),
            "permission_scope": "Git executable bit only; tar checks regular mode against existing inventory; uid/gid not restored"}, repo


def map_readability(repo):
    import yaml
    from PIL import Image
    folder = repo / "ws/src/maps/d1_qazbot_20261029"
    cfg = yaml.safe_load((folder / "map.yaml").read_text())
    with Image.open(folder / cfg["image"]) as image:
        image.load()
        pixels = collections.Counter(image.tobytes())
        unknown = sum(n for v, n in pixels.items()
                      if cfg["free_thresh"] <= (255-v)/255 <= cfg["occupied_thresh"])
        return {"status": "OFFLINE_VERIFIED", "size": list(image.size), "mode": image.mode,
                "pixels": dict(pixels), "unknown_cells_trinary": unknown,
                "resolution": cfg["resolution"], "origin": cfg["origin"],
                "pgm_sha256": file_hash(folder / cfg["image"]),
                "yaml_sha256": file_hash(folder / "map.yaml"),
                "scope": "YAML and PGM decoding only; no map_server or ROS graph"}


def backend_imports(repo):
    # Import known library/guarded modules, never scripts with top-level I/O.
    code = """import importlib,sys,json
from pathlib import Path
r=Path(sys.argv[1]);sys.dont_write_bytecode=True
sys.path[:0]=[str(r/'tools/nightly'),str(r/'ws/src/navigation'),str(r/'ws/src/lio'),str(r/'tests/collision_monitor/lib'),str(r/'tests/d04')]
out={}
for name in ['loc_trust','control_geometry','native_odom','level_bridge','native_odom_bridge','goal_coordinator','x2_motion_adapter','costmap_emul','replay_d04','cm_eval_lib','numpy','scipy','yaml','PIL','rclpy','rosbag2_py','sensor_msgs_py','aimdk_msgs']:
 try:
  m=importlib.import_module(name);out[name]={'status':'OFFLINE_VERIFIED','file':getattr(m,'__file__',None)}
 except Exception as e:out[name]={'status':'BLOCKED','error':type(e).__name__+': '+str(e)}
print(json.dumps(out))
"""
    environment = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
    # ROS setup only changes process environment; no node is constructed.
    result = run(["bash", "-c", 'source /opt/ros/humble/setup.bash && exec "$@"', "restore-imports",
                  sys.executable, "-c", code, str(repo)], env=environment)
    # Also verify current LAB backend imports separately; these are not part of
    # the historical robot preservation snapshot.
    lab_code = code.replace("['loc_trust',", "['source_state','lab','labctl','preflight','loc_trust',")
    lab_result = run(["bash", "-c", 'source /opt/ros/humble/setup.bash && exec "$@"', "lab-imports",
                      sys.executable, "-c", lab_code, str(ROOT)], env=environment)
    return {"process": result, "modules": json.loads(result["stdout"]) if result["returncode"] == 0 else {},
            "current_lab_process": lab_result,
            "current_lab_modules": json.loads(lab_result["stdout"]) if lab_result["returncode"] == 0 else {}}


def projector(repo, scratch):
    build = scratch / "cpp_build"
    install = scratch / "cpp_install"
    package = repo / "ws/src/qazbot_lidar_projector"
    # Compile only, BUILD_TESTING=OFF prevents launch_testing graph execution.
    cmd = ["bash", "-c", 'source /opt/ros/humble/setup.bash && exec "$@"', "restore-build",
           "cmake", "-S", str(package), "-B", str(build), "-DBUILD_TESTING=OFF",
           "-DCMAKE_BUILD_TYPE=Release", "-DPython3_EXECUTABLE=/usr/bin/python3",
           "-DCMAKE_INSTALL_PREFIX=" + str(install)]
    configure = run(cmd)
    result = {"configure": configure, "status": "BLOCKED"}
    if configure["returncode"] == 0:
        result["build"] = run(["cmake", "--build", str(build), "--parallel", "1"])
        if result["build"]["returncode"] == 0:
            result["install"] = run(["cmake", "--install", str(build)])
            result["status"] = "OFFLINE_VERIFIED" if result["install"]["returncode"] == 0 else "BLOCKED"
    return result


def bags(source):
    import re
    import yaml
    manifest = ROOT / "tests/d04/BAGS.md"
    entries = re.findall(r"\| (d04_static_bag_\d+) .*?\| `([^`]+\.mcap)` \| (\d+) \| `([0-9a-f]{64})`", manifest.read_text())
    roots = [ROOT / "docs/evidence/nav_20261102_live", source,
             Path("/home/adn/QazBot/control_backend/docs/evidence/nav_20261102_live")]
    if os.environ.get("QAZBOT_D04_BAGS"):
        roots.insert(0, Path(os.environ["QAZBOT_D04_BAGS"]))
    if os.environ.get("QAZBOT_D04_COPY_ROOTS"):
        roots = [Path(p) for p in os.environ["QAZBOT_D04_COPY_ROOTS"].split(os.pathsep)] + roots
    result = []
    for name, rel, size, sha in entries:
        copies = []
        for root in roots:
            path = root / rel
            try:
                s = path.stat()
                copies.append({"path": str(path), "realpath": str(path.resolve()), "device": s.st_dev,
                               "inode": s.st_ino, "size": s.st_size, "sha256": file_hash(path)})
                try:
                    metadata = path.parent / "metadata.yaml"
                    data = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
                    copies[-1]["metadata"] = {"sha256": file_hash(metadata),
                                              "readable": path.name in data["relative_file_paths"],
                                              "storage_identifier": data.get("storage_identifier")}
                except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
                    copies[-1]["metadata"] = {"readable": False, "error": str(exc)}
            except OSError as exc:
                copies.append({"path": str(path), "error": str(exc)})
        valid = [c for c in copies if c.get("sha256") == sha and c.get("size") == int(size)
                 and c.get("metadata", {}).get("readable")]
        identities = {(c["device"], c["inode"]) for c in valid}
        result.append({"name": name, "relative_path": rel, "expected_size": int(size),
                       "expected_sha256": sha, "checks": copies,
                       "independent_file_copies": len(identities),
                       "distinct_filesystems": len({c["device"] for c in valid}),
                       "status": "OFFLINE_VERIFIED" if len(identities) >= 2 else "BLOCKED"})
    if len(result) != 3:
        raise ValueError("expected all three bags in existing BAGS.md")
    return result


def verify(source, scratch):
    inventory = {x["path"]: x for x in json.loads((source / "inventory/files.json").read_text())}
    free = shutil.disk_usage(scratch)
    report = {"utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "classification": "OFFLINE_VERIFIED", "source": str(source),
              "initial_head": git(ROOT, "rev-parse", "HEAD").strip(),
              "resources_before": {"free_bytes": free.free, "total_bytes": free.total},
              "input_manifests": {str(p.relative_to(source)): file_hash(p) for p in
                                  [source / "inventory/files.json", source / "SHA256SUMS", source / "final_deployment_validation.json"]}}
    report["archives"] = archives(source, inventory, scratch)
    report["bundle_restore"], repo = restore_bundle(source, inventory, scratch)
    if repo:
        report["d1_map"] = map_readability(repo)
        report["backend_imports"] = backend_imports(repo)
        report["projector"] = projector(repo, scratch)
    report["bags"] = bags(source)
    private = report["archives"]["robot_code_runtime_private.tar.gz"]
    report["dependencies"] = {
        "inventory_errors": [r for r in inventory.values() if "error" in r],
        "missing_archive_link_targets": [x for x in private.get("links", []) if x["resolution"] != "archived"],
        "historical_partial": report["archives"]["historical_preservation.tar"].get("partial_files", []),
        "not_imaged": ["/opt (ROS/system dependencies)", "/agibot (vendor runtime/firmware)"],
        "sdk": "aimdk tree is in the private tar; host import is a separate dependency check, not an absent-backup claim",
        "scope": "existing inventory/REPORT.md only, no robot inventory or execution of archived binaries"}
    report["blocked"] = []
    for name, value in report["archives"].items():
        if value["status"] != "OFFLINE_VERIFIED":
            report["blocked"].append("archive:" + name)
    for section in ("bundle_restore", "d1_map", "projector"):
        if report.get(section, {}).get("status") != "OFFLINE_VERIFIED":
            report["blocked"].append(section)
    report["blocked"].extend("independent_copies:" + x["name"] for x in report["bags"] if x["status"] == "BLOCKED")
    report["blocked"].extend("restored_import:" + name for name, value in
                             report.get("backend_imports", {}).get("modules", {}).items() if value["status"] == "BLOCKED")
    if report["dependencies"]["missing_archive_link_targets"]:
        report["blocked"].append("runtime_archive_external_or_broken_symlinks")
    if report["dependencies"]["historical_partial"]:
        report["blocked"].append("incomplete_historical_backup")
    if report["dependencies"]["not_imaged"]:
        report["blocked"].append("system_and_vendor_dependencies_not_imaged")
    report["status"] = "RESTORE_BLOCKED" if report["blocked"] else "RESTORE_VERIFIED"
    report["d05_independent_work"] = "UNBLOCKED for offline safety work; bag absence is not a D05 dependency"
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--scratch", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="restore-", dir=args.scratch) as temp:
        report = verify(args.source, Path(temp))
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "evidence": str(args.output)}))


if __name__ == "__main__":
    main()
