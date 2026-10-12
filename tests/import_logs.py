#!/usr/bin/env python3
"""Import robot logs from the SD card into tests/logs/, one file per boot.

The brain writes one file per boot to /usd/logs/boot_NNNN.txt (see
src/utilities/logger.cpp). This pulls every boot that is not already in
tests/logs/, names it <MMDDYYYY>-<boot>.txt (import date + boot number, so it
slots into the existing naming), prints a one-line summary of each, and can
open the new files in VS Code. Run it as often as you like: boots already
imported are skipped (tracked by content hash in tests/logs/.imported.json).

Older single-file logs (robot_log_*.txt, where every boot appended to one
file) are split on the "Logger initialized successfully" line each boot
writes first, and imported as <MMDDYYYY>-legacy<N>.txt.

    python3 tests/import_logs.py            # import what is new
    python3 tests/import_logs.py --open     # ...and open the new files in VS Code
    python3 tests/import_logs.py --archive  # ...and move them to logs/imported/ on the card
    python3 tests/import_logs.py --list     # what is on the card, and what is already in
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_CARD = Path("/mnt/sdcard")
DEFAULT_LOG_DIR = HERE / "logs"
REGISTRY_NAME = ".imported.json"

BOOT_FILE_RE = re.compile(r"^boot_(\d+)\.txt$")
LEGACY_FILE_RE = re.compile(r"^robot_log_.*\.txt$")
BOOT_START_RE = re.compile(r"^\[\d{2}:\d{2}:\d{2}\.\d{3}\] \[INFO\] Logger initialized successfully")
LINE_RE = re.compile(r"^\[(\d{2}):(\d{2}):(\d{2})\.(\d{3})\] \[(\w+)\] (.*)$")
ROUTE_RE = re.compile(r"trajectory '([^']*)' loaded")
PROGRAM_RE = re.compile(r"^Program type: (\S+)")


# --- card side -------------------------------------------------------------------


@dataclass
class CardLog:
    path: Path
    kind: str  # "boot" or "legacy"
    boot: int | None = None  # boot number for kind == "boot"


def find_card_logs(card: Path) -> list[CardLog]:
    """Boot files (logs/boot_NNNN.txt) by number, then legacy dumps."""
    found: list[CardLog] = []
    logs_dir = card / "logs"
    if logs_dir.is_dir():
        for p in logs_dir.iterdir():
            m = BOOT_FILE_RE.match(p.name)
            if m and p.is_file():
                found.append(CardLog(p, "boot", int(m.group(1))))
    found.sort(key=lambda c: c.boot or 0)
    legacy: list[CardLog] = []
    for folder in (card, logs_dir):
        if folder.is_dir():
            for p in sorted(folder.iterdir()):
                if LEGACY_FILE_RE.match(p.name) and p.is_file():
                    legacy.append(CardLog(p, "legacy"))
    return found + legacy


def split_boots(text: str) -> list[str]:
    """Split a legacy multi-boot dump into one chunk per boot.

    Every boot's first line is "Logger initialized successfully". Anything
    before the first marker (should not happen) becomes its own chunk so no
    lines are dropped.
    """
    chunks: list[list[str]] = []
    for line in text.splitlines(keepends=True):
        if BOOT_START_RE.match(line) or not chunks:
            chunks.append([])
        chunks[-1].append(line)
    return ["".join(c) for c in chunks if "".join(c).strip()]


# --- summaries -------------------------------------------------------------------


@dataclass
class Summary:
    lines: int = 0
    duration_s: float = 0.0
    program: str = "?"
    route: str = "?"
    runs: int = 0
    errors: int = 0
    warnings: int = 0
    streams: list[str] = field(default_factory=list)

    def __str__(self) -> str:
        tags = " ".join(f"[{s}]" for s in self.streams)
        return (
            f"{self.program:<11} route={self.route:<20} runs={self.runs:<2} "
            f"{self.duration_s:6.1f}s {self.lines:>7,} lines "
            f"err={self.errors} warn={self.warnings} {tags}".rstrip()
        )


def summarize(text: str) -> Summary:
    s = Summary()
    streams = set()
    for line in text.splitlines():
        s.lines += 1
        m = LINE_RE.match(line)
        if not m:
            continue
        h, mi, sec, ms, level, msg = m.groups()
        s.duration_s = max(s.duration_s, int(h) * 3600 + int(mi) * 60 + int(sec) + int(ms) / 1000)
        if level == "ERROR":
            s.errors += 1
        elif level == "WARNING":
            s.warnings += 1
        pm = PROGRAM_RE.match(msg)
        if pm:
            s.program = pm.group(1)
        rm = ROUTE_RE.search(msg)
        if rm:
            s.route = rm.group(1)
        if msg.startswith("Initial pose:"):
            s.runs += 1
        if msg.startswith("LIFTDBG"):
            streams.add("lift")
        if msg.startswith("DRIVEHEALTH"):
            streams.add("health")
        if msg.startswith("ROLLERDBG"):
            streams.add("roller")
    s.streams = sorted(streams)
    return s


# --- importing -------------------------------------------------------------------


def sha256(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8", "surrogateescape")).hexdigest()


def load_registry(log_dir: Path) -> dict:
    path = log_dir / REGISTRY_NAME
    if path.is_file():
        try:
            return json.loads(path.read_text())
        except json.JSONDecodeError:
            print(f"warning: {path} is corrupt; starting a fresh registry", file=sys.stderr)
    return {}


def save_registry(log_dir: Path, registry: dict) -> None:
    (log_dir / REGISTRY_NAME).write_text(json.dumps(registry, indent=2, sort_keys=True) + "\n")


def unique_name(log_dir: Path, stem: str) -> str:
    """<stem>.txt, or <stem>b.txt, <stem>c.txt... if a different file is there."""
    name = f"{stem}.txt"
    suffix = ord("b")
    while (log_dir / name).exists():
        name = f"{stem}{chr(suffix)}.txt"
        suffix += 1
    return name


def next_legacy_index(log_dir: Path, date: str) -> int:
    n = 0
    for p in log_dir.glob(f"{date}-legacy*.txt"):
        m = re.match(rf"^{re.escape(date)}-legacy(\d+)", p.name)
        if m:
            n = max(n, int(m.group(1)))
    return n + 1


@dataclass
class Imported:
    dest: Path
    source: Path
    summary: Summary
    label: str  # "boot 7" / "legacy 1"


@dataclass
class Unreadable:
    """A card file the kernel refused to hand over, and how far it got."""

    source: CardLog
    error: OSError
    readable_bytes: int


def readable_prefix_size(path: Path, chunk: int = 32 * 1024) -> int:
    """Bytes readable from the start of `path` before the card gives up.

    A FAT chain that goes bad mid-file still serves the clusters before the
    break, so this says how much of a corrupt boot could be salvaged by hand.
    """
    total = 0
    try:
        with path.open("rb") as fh:
            while data := fh.read(chunk):
                total += len(data)
    except OSError:
        pass
    return total


def import_new(
    card: Path,
    log_dir: Path,
    date: str,
    archive: bool = False,
    delete: bool = False,
    dry_run: bool = False,
) -> tuple[list[Imported], list[CardLog], list[Unreadable]]:
    """Copy every boot on the card that is not yet in log_dir.

    Returns (imported, skipped, unreadable). With archive=True the source file
    is moved to <card>/logs/imported/ afterwards; with delete=True it is
    removed. A file the card cannot serve (corrupt FAT cluster chain, usually
    a boot cut short by power loss) is reported in `unreadable` and skipped,
    so one bad file does not strand the boots that follow it.
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    registry = load_registry(log_dir)
    known = {entry["sha256"] for entry in registry.values()}
    imported: list[Imported] = []
    skipped: list[CardLog] = []
    unreadable: list[Unreadable] = []
    legacy_index = next_legacy_index(log_dir, date)

    for card_log in find_card_logs(card):
        try:
            text = card_log.path.read_text(encoding="utf-8", errors="surrogateescape")
        except OSError as exc:
            unreadable.append(Unreadable(card_log, exc, readable_prefix_size(card_log.path)))
            continue
        if card_log.kind == "boot":
            chunks = [(text, f"{date}-{card_log.boot}", f"boot {card_log.boot}")]
        else:
            chunks = []
            for chunk in split_boots(text):
                chunks.append((chunk, f"{date}-legacy{legacy_index}", f"legacy {legacy_index}"))
                legacy_index += 1
        new_here = 0
        for chunk, stem, label in chunks:
            digest = sha256(chunk)
            if digest in known:
                if card_log.kind == "legacy":
                    legacy_index -= 1  # number not consumed
                continue
            name = unique_name(log_dir, stem)
            dest = log_dir / name
            if not dry_run:
                dest.write_text(chunk, encoding="utf-8", errors="surrogateescape")
                registry[name] = {
                    "sha256": digest,
                    "source": str(card_log.path.relative_to(card)),
                    "imported": dt.datetime.now().isoformat(timespec="seconds"),
                }
            known.add(digest)
            imported.append(Imported(dest, card_log.path, summarize(chunk), label))
            new_here += 1
        if new_here == 0:
            skipped.append(card_log)
        elif not dry_run and (archive or delete):
            if delete:
                card_log.path.unlink()
            else:
                archive_dir = card / "logs" / "imported"
                archive_dir.mkdir(parents=True, exist_ok=True)
                shutil.move(str(card_log.path), str(archive_dir / card_log.path.name))

    if not dry_run and imported:
        save_registry(log_dir, registry)
    return imported, skipped, unreadable


# --- CLI -------------------------------------------------------------------------


def _list(card: Path, log_dir: Path) -> int:
    registry = load_registry(log_dir)
    by_source = {}
    for name, entry in registry.items():
        by_source.setdefault(entry["source"], []).append(name)
    logs = find_card_logs(card)
    if not logs:
        print(f"no logs on the card at {card} (looked in logs/boot_*.txt and robot_log_*.txt)")
        return 0
    for c in logs:
        rel = str(c.path.relative_to(card))
        label = f"boot {c.boot}" if c.kind == "boot" else "legacy"
        status = ", ".join(sorted(by_source.get(rel, []))) or "NEW"
        size = c.path.stat().st_size
        print(f"{rel:<32} {label:<10} {size:>9,} B  -> {status}")
    return 0


def report_unreadable(unreadable: list[Unreadable]) -> int:
    """Print what the card could not serve. Returns the process exit code."""
    if not unreadable:
        return 0
    sys.stdout.flush()  # keep the warning after the import summary on a terminal
    print(f"\nwarning: {len(unreadable)} file(s) could not be read off the card:", file=sys.stderr)
    for bad in unreadable:
        size = bad.source.path.stat().st_size
        print(
            f"  {bad.source.path}: {bad.error.strerror or bad.error}"
            f" ({bad.readable_bytes:,} of {size:,} B readable)",
            file=sys.stderr,
        )
    print(
        "The card's filesystem is damaged, not the log. Check `dmesg | tail` for the\n"
        "FAT error, salvage what is readable with\n"
        "  dd if=<file> of=<dest> bs=512 conv=noerror,sync\n"
        "then repair the card: `sdcard eject`, fsck.vfat -a the device (or chkdsk /f on\n"
        "Windows), and re-insert it. A FAT error also remounts the card read-only, so\n"
        "exports will fail until it is re-mounted.",
        file=sys.stderr,
    )
    return 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--card", type=Path, default=DEFAULT_CARD, help="SD card mount point (default: /mnt/sdcard)")
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="destination (default: tests/logs)")
    ap.add_argument("--date", default=dt.date.today().strftime("%m%d%Y"), help="MMDDYYYY prefix for new files (default: today)")
    ap.add_argument("--open", action="store_true", help="open the imported files in VS Code")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--archive", action="store_true", help="move imported files to logs/imported/ on the card")
    group.add_argument("--delete", action="store_true", help="delete imported files from the card")
    ap.add_argument("--dry-run", action="store_true", help="show what would be imported, change nothing")
    ap.add_argument("--list", action="store_true", help="list card logs and their import status, then exit")
    args = ap.parse_args(argv)

    if not args.card.is_dir() or not any(args.card.iterdir()):
        print(f"error: nothing mounted at {args.card}. Insert the card, or run: sdcard status", file=sys.stderr)
        return 2
    if args.list:
        return _list(args.card, args.log_dir)

    imported, skipped, unreadable = import_new(
        args.card, args.log_dir, args.date, args.archive, args.delete, args.dry_run
    )
    verb = "would import" if args.dry_run else "imported"
    if not imported:
        print(f"nothing new on the card ({len(skipped)} file(s) already imported)")
        return report_unreadable(unreadable)
    print(f"{verb} {len(imported)} boot(s) into {args.log_dir}:")
    for item in imported:
        print(f"  {item.dest.name:<24} {item.label:<10} {item.summary}")
    if skipped:
        print(f"skipped {len(skipped)} file(s) already imported")
    first = imported[0].dest.name
    print("\nnext:")
    print(f"  python3 tests/visualize.py dashboard {first} --run all")
    print(f"  python3 tests/run_analysis.py {first}")
    if args.open and not args.dry_run:
        try:
            subprocess.run(["code", *[str(i.dest) for i in imported]], check=False)
        except FileNotFoundError:
            print("warning: 'code' not on PATH; open the files by hand", file=sys.stderr)
    return report_unreadable(unreadable)


if __name__ == "__main__":
    sys.exit(main())
