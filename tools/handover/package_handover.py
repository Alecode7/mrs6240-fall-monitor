"""Create a private R&D snapshot, history bundles, verification reports and ZIP.

This tool only writes new destinations. It never deletes or modifies source files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime, timedelta, timezone
import zipfile


ROOT = Path(__file__).resolve().parents[2]
SKIP_DIRS = {".git", ".git.local-backup", ".venv", "__pycache__", ".pytest_cache", ".cache", ".cdk", ".mplconfig"}
CORE_TESTS = [
    "test_runtime_paths.py", "test_device_source.py", "test_serial_event.py",
    "test_serial_event_live_visualizer.py", "test_firmware_falldown_contract.py",
]


def say(message):
    print(message, flush=True)


def long_path(path):
    path = Path(path).resolve()
    if os.name == "nt" and not str(path).startswith("\\\\?\\"):
        return Path("\\\\?\\" + str(path))
    return path


def dump(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def sha256(path):
    digest = hashlib.sha256()
    with long_path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def ignore_local(directory, names):
    path = Path(str(directory).removeprefix("\\\\?\\"))
    excluded = {name for name in names if name in SKIP_DIRS or name.endswith((".pyc", ".pyo"))}
    if path == ROOT:
        excluded.update({name for name in names if name in {".build", "releases"} or name.endswith(".obj")})
    # Firmware compilation caches, never SDK libraries or post-build scripts.
    if "Falldown" in path.parts:
        excluded.update({name for name in names if name in {"Obj", "Lst"}})
    return excluded


def run(command, *, cwd=None, env=None, timeout=None):
    return subprocess.run(command, cwd=cwd, env=env, timeout=timeout,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          encoding="utf-8", errors="replace")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-parent", type=Path, default=ROOT.parent / "handover")
    parser.add_argument("--toolchain", type=Path, default=Path("D:/C-Sky/CDKRepo/Toolchain/XTGccElfNewlib/V3.1.0/R"))
    args = parser.parse_args()
    timestamp = datetime.now(timezone(timedelta(hours=8)))
    name = "MRS6240-" + timestamp.strftime("%Y%m%d-%H%M%S")
    parent = args.output_parent.resolve()
    if parent == ROOT or ROOT in parent.parents:
        raise ValueError("Output must be outside the original project.")
    package = parent / name
    archive = package.with_suffix(".zip")
    if package.exists() or archive.exists():
        raise FileExistsError(package)
    toolchain = args.toolchain.resolve()
    if not (toolchain / "bin/riscv64-unknown-elf-gcc.exe").is_file():
        raise FileNotFoundError(toolchain)
    parent.mkdir(parents=True, exist_ok=True)
    package.mkdir()
    say(f"PACKAGE={package}")
    say("1/7 Copying complete project snapshot (SDK, data, logs, source)...")
    project = package / "project"
    shutil.copytree(long_path(ROOT), long_path(project), ignore=ignore_local)
    complete_releases = sorted(
        (path for path in (ROOT / "releases").glob("*/MRS6240-Fall-Monitor")
         if (path / "build-manifest.json").is_file()
         and (path / "MRS6240Monitor.exe").is_file()
         and (path / "MRS6240Engineering.exe").is_file()), reverse=True)
    if not complete_releases:
        raise FileNotFoundError("No complete portable release")
    release = complete_releases[0]
    copied_release = project / release.relative_to(ROOT)
    shutil.copytree(long_path(release), long_path(copied_release), ignore=ignore_local)
    say("2/7 Preserving compiler, firmware and Git history...")
    shutil.copytree(long_path(toolchain), long_path(package / "toolchains/XTGccElfNewlib/V3.1.0/R"), ignore=ignore_local)
    firmware = ROOT / "SDK/Software_Kit/02_SDK/psdf_sdk/project/mmwave/mmwave_application/Falldown/cdk_6240_cpuf"
    (package / "firmware/current").mkdir(parents=True)
    shutil.copy2(firmware / "output/mrs6240_p2512.img", package / "firmware/current/mrs6240_p2512.img")
    shutil.copytree(firmware / "firmware_archive", package / "firmware/archive")
    history = package / "history"
    history.mkdir()
    for label, git_dir in [("project", ROOT / ".git"),
                           ("legacy-fall-detection", ROOT / "fall_detection/.git.local-backup"),
                           ("legacy-capture-station", ROOT / "fall_capture_station/.git.local-backup")]:
        if git_dir.is_dir():
            result = run(["git", f"--git-dir={git_dir}", "bundle", "create", str(history / f"{label}.bundle"), "--all"])
            if result.returncode:
                raise RuntimeError(result.stdout)
    result = run(["git", "-C", str(ROOT), "diff", "--binary", f"--output={history / 'local-tracked-changes.patch'}", "HEAD"])
    if result.returncode:
        raise RuntimeError(result.stdout)
    (history / "working-tree-status.txt").write_text(run(["git", "-C", str(ROOT), "-c", "core.quotepath=false", "status", "--short", "--untracked-files=all"]).stdout, encoding="utf-8")
    (history / "commits.txt").write_text(run(["git", "-C", str(ROOT), "log", "--all", "--date=iso", "--format=%H %ad %s"]).stdout, encoding="utf-8")
    shutil.copy2(ROOT / "docs/HANDOVER_20261119.md", package / "START_HERE.md")
    shutil.copy2(ROOT / "tools/handover/VERIFY.ps1", package / "VERIFY.ps1")

    environment = package / "environment"
    environment.mkdir()
    freeze = run([sys.executable, "-m", "pip", "freeze"])
    if freeze.returncode:
        raise RuntimeError(freeze.stdout)
    (environment / "requirements-pc-lock.txt").write_text(freeze.stdout, encoding="utf-8")
    compiler = run([str(package / "toolchains/XTGccElfNewlib/V3.1.0/R/bin/riscv64-unknown-elf-gcc.exe"), "--version"])
    if compiler.returncode:
        raise RuntimeError(compiler.stdout)
    dump(environment / "versions.json", {
        "python": sys.version, "cdk_version": "2.24.11", "cdk_included": False,
        "sdk_version": "2.1.0 (SDK/Release_Notes.txt)", "toolchain_package": "XTGccElfNewlib/V3.1.0/R",
        "compiler": compiler.stdout, "portable_release": release.parent.name,
        "original_source_directory": str(ROOT), "venvs_included": False,
        "machine_proxy_or_git_config_included": False,
    })
    say("3/7 Testing the copied source snapshot...")
    verification = package / "verification"
    verification.mkdir()
    env = os.environ.copy()
    env["PYTHONPATH"] = str(project)
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    tests = run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", *[f"fall_detection/tests/{name}" for name in CORE_TESTS],
                 f"--junitxml={verification / 'pytest.xml'}"], cwd=project, env=env, timeout=120)
    (verification / "pytest.txt").write_text(tests.stdout, encoding="utf-8")
    say(tests.stdout.strip().splitlines()[-1])
    say("4/7 Checking copied executables without opening hardware...")
    exe_env = os.environ.copy()
    exe_env["PATH"] = str(Path(os.environ["WINDIR"]) / "System32") + os.pathsep + os.environ["WINDIR"]
    exe_env.pop("PYTHONPATH", None)
    exe_env.pop("PYTHONHOME", None)
    checks = []
    for exe, switch, report in [("MRS6240Monitor.exe", "--self-test", "exe-dependencies.json"),
                                 ("MRS6240Monitor.exe", "--smoke-test", "monitor-ui.json"),
                                 ("MRS6240Engineering.exe", "--smoke-test", "engineering-ui.json")]:
        report_path = verification / report
        result = run([str(copied_release / exe), switch, str(report_path)], cwd=copied_release, env=exe_env, timeout=30)
        if result.returncode or not report_path.is_file():
            raise RuntimeError(f"EXE validation failed: {exe} {switch}: {result.stdout}")
        check = json.loads(report_path.read_text(encoding="utf-8"))
        if not check.get("passed"):
            raise RuntimeError(f"EXE validation failed: {report}")
        checks.append({"executable": exe, "check": switch, "passed": True, "report": report})
        say(f"PASS {exe} {switch}")
    for bundle in history.glob("*.bundle"):
        check = run(["git", "-C", str(ROOT), "bundle", "verify", str(bundle)])
        (verification / (bundle.stem + "-bundle.txt")).write_text(check.stdout, encoding="utf-8")
        if check.returncode:
            raise RuntimeError(check.stdout)
    dump(verification / "summary.json", {"pytest_exit_code": tests.returncode,
        "known_failed_test": "test_firmware_macros_match_validated_python_profile (historical 163cm/30deg profile)",
        "exe_checks": checks, "firmware_rebuilt": False, "hardware_opened_by_verification": False})

    say("5/7 Generating per-file SHA256 manifest...")
    files = []
    extended_package = long_path(package)
    for index, path in enumerate(sorted(path for path in extended_package.rglob("*") if path.is_file()), 1):
        files.append({"path": path.relative_to(extended_package).as_posix(), "bytes": path.stat().st_size, "sha256": sha256(path)})
        if index % 2000 == 0:
            say(f"Hashed {index} files")
    dump(package / "MANIFEST.json", {"created_at": timestamp.isoformat(), "file_count": len(files),
        "total_bytes": sum(entry["bytes"] for entry in files),
        "excluded": sorted(SKIP_DIRS | {"root .build", "incomplete portable release", "Falldown Obj/Lst", "root *.obj"}),
        "manifest_excludes_itself": True, "files": files})
    say("6/7 Creating ZIP archive...")
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED, compresslevel=6, allowZip64=True) as zipped:
        for index, path in enumerate(sorted(path for path in extended_package.rglob("*") if path.is_file()), 1):
            zipped.write(path, (Path(name) / path.relative_to(extended_package)).as_posix())
            if index % 2000 == 0:
                say(f"Archived {index} files")
    say("7/7 Reading complete archive and validating CRC...")
    with zipfile.ZipFile(archive) as zipped:
        bad = zipped.testzip()
        if bad:
            raise RuntimeError(f"Corrupt ZIP entry: {bad}")
        if len(zipped.namelist()) != len(files) + 1:
            raise RuntimeError("Archive file count does not match manifest")
    archive_hash = sha256(archive)
    archive.with_suffix(".zip.sha256").write_text(f"{archive_hash}  {archive.name}\n", encoding="utf-8")
    say(json.dumps({"package": str(package), "archive": str(archive), "zip_bytes": archive.stat().st_size,
                    "source_files": len(files), "uncompressed_bytes": sum(entry["bytes"] for entry in files),
                    "sha256": archive_hash, "zip_crc_check": "passed"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
