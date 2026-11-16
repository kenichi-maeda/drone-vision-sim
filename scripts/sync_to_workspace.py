#!/usr/bin/env python3
"""Copy the drone-side files of this checkout into the workspace devcontainer.

The Duckietown workbench builds from the project copy inside the workspace
devcontainer (/home/ubuntu/drone-vision-sim), which is a separate checkout
from this one on the Mac. This script copies only what the drone side needs,
comparing checksums so that unchanged files are left alone.

    python3 scripts/sync_to_workspace.py --dry-run          # show what would change
    python3 scripts/sync_to_workspace.py                    # copy added/changed files
    python3 scripts/sync_to_workspace.py --delete           # also remove files deleted here
    python3 scripts/sync_to_workspace.py --only recipe/launchers/overwatch.sh maps/overwatch_arena

Files that exist only in the workspace copy are never removed without --delete.
Nothing is committed on either side.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import subprocess
import sys
import tarfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEFAULT_PATHS = (
    "packages/drone_vision",
    "recipe",
    "scripts",
    "patches",
    "maps/overwatch_arena",
    "assets",
)
SKIP_DIRS = {"__pycache__", ".pytest_cache", ".mypy_cache"}
SKIP_SUFFIXES = (".pyc",)
SKIP_NAMES = {".DS_Store"}


def skipped(rel: str) -> bool:
    parts = rel.split("/")
    return (
        any(p in SKIP_DIRS for p in parts)
        or parts[-1] in SKIP_NAMES
        or rel.endswith(SKIP_SUFFIXES)
    )


def local_files(paths: list[str]) -> dict[str, str]:
    files: dict[str, str] = {}
    for top in paths:
        root = REPO / top
        candidates = [root] if root.is_file() else sorted(root.rglob("*")) if root.is_dir() else []
        for path in candidates:
            if not path.is_file():
                continue
            rel = path.relative_to(REPO).as_posix()
            if not skipped(rel):
                files[rel] = hashlib.md5(path.read_bytes()).hexdigest()
    return files


def docker(container: str, args: list[str], *, user: str = "ubuntu", stdin: bytes | None = None) -> bytes:
    cmd = ["docker", "exec", "-i", "-u", user, container, *args]
    result = subprocess.run(cmd, input=stdin, capture_output=True)
    if result.returncode != 0:
        sys.exit(f"Command failed in {container}: {' '.join(args)}\n{result.stderr.decode(errors='replace')}")
    return result.stdout


def remote_files(container: str, dest: str, paths: list[str]) -> dict[str, str]:
    script = (
        'cd "$1" || exit 3; shift; '
        'for p in "$@"; do [ -e "$p" ] && find "$p" -type f -print0; done | xargs -0 -r md5sum'
    )
    out = docker(container, ["sh", "-c", script, "sh", dest, *paths]).decode()
    files: dict[str, str] = {}
    for line in out.splitlines():
        digest, _, rel = line.partition("  ")
        if rel and not skipped(rel):
            files[rel] = digest
    return files


def make_tar(rels: list[str]) -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w", format=tarfile.GNU_FORMAT) as tar:
        for rel in rels:
            info = tar.gettarinfo(str(REPO / rel), arcname=rel)
            info.uid = info.gid = 0
            info.uname = info.gname = ""
            with open(REPO / rel, "rb") as handle:
                tar.addfile(info, handle)
    return buffer.getvalue()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="only report what would change")
    parser.add_argument("--delete", action="store_true",
                        help="remove files under the synced paths that no longer exist here")
    parser.add_argument("--only", nargs="+", metavar="PATH",
                        help="sync only these files or directories (relative to the repo root)")
    parser.add_argument("--container", default="workspace_devcontainer-workspace-1")
    parser.add_argument("--dest", default="/home/ubuntu/drone-vision-sim")
    args = parser.parse_args()

    if args.only:
        paths = args.only
        missing = [rel for rel in paths if not (REPO / rel).exists()]
        if missing:
            sys.exit(f"Not found in this checkout: {', '.join(missing)}")
    else:
        paths = [rel for rel in DEFAULT_PATHS if (REPO / rel).exists()]

    local = local_files(paths)
    remote = remote_files(args.container, args.dest, paths)
    added = sorted(set(local) - set(remote))
    changed = sorted(r for r in set(local) & set(remote) if local[r] != remote[r])
    removed = sorted(set(remote) - set(local)) if args.delete else []
    remote_only = sorted(set(remote) - set(local))

    print(f"Sync {REPO} -> {args.container}:{args.dest}")
    print(f"Paths: {' '.join(paths)}")
    for label, rels in (("add", added), ("update", changed), ("delete", removed)):
        for rel in rels:
            print(f"  {label:<6} {rel}")
    if remote_only and not args.delete:
        print(f"  ({len(remote_only)} file(s) exist only in the workspace copy; kept. Use --delete to remove.)")
    if not (added or changed or removed):
        print("Already in sync.")
        return
    if args.dry_run:
        print(f"Dry run: {len(added)} to add, {len(changed)} to update, {len(removed)} to delete. Nothing copied.")
        return

    to_copy = added + changed
    if to_copy:
        docker(args.container, ["tar", "-xf", "-", "-C", args.dest, "--no-same-owner"], stdin=make_tar(to_copy))
    if removed:
        docker(args.container, ["rm", "-f", "--", *[f"{args.dest}/{rel}" for rel in removed]])
    after = remote_files(args.container, args.dest, paths)
    mismatched = [rel for rel in to_copy if after.get(rel) != local[rel]]
    if mismatched:
        sys.exit(f"Copied, but checksums differ for: {', '.join(mismatched)}")
    print(f"Done: {len(added)} added, {len(changed)} updated, {len(removed)} deleted.")


if __name__ == "__main__":
    main()
