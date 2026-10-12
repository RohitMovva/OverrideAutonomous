"""Tests for tests/import_logs.py (pulling per-boot logs off the SD card).

Uses a fake card in tmp_path; nothing touches /mnt/sdcard.
    pytest tests/test_import_logs.py -v
"""

import errno
import json
from pathlib import Path

import pytest

import import_logs as il

BOOT_A = (
    "[00:00:00.000] [INFO] Logger initialized successfully\n"
    "[00:00:00.000] [INFO] Log file: /usd/logs/boot_0007.txt\n"
    "[00:00:00.500] [LOG] init: trajectory 'threepin' loaded size=1422, usedHeap=1\n"
    "[00:00:00.900] [LOG] Robot initialized\n"
    "[00:00:02.100] [LOG] Program type: autonomous\n"
    "[00:00:02.100] [LOG] Initial pose: -60.6 0.0 -3.14\n"
    "[00:00:03.250] [WARNING] something odd\n"
    "[00:00:12.400] [LOG] DRIVEHEALTH auton L0 port=-18 tempC=35 curMA=810 W=5.2 eff=71 ovt=0\n"
)
BOOT_B = (
    "[00:00:00.000] [INFO] Logger initialized successfully\n"
    "[00:00:00.000] [INFO] Log file: /usd/logs/boot_0008.txt\n"
    "[00:00:01.000] [LOG] Program type: opcontrol\n"
    "[00:00:01.000] [ERROR] init: drive encoders did NOT tare\n"
    "[00:00:05.000] [LOG] LIFTDBG target=1 actual=2\n"
)


@pytest.fixture
def card(tmp_path):
    c = tmp_path / "card"
    (c / "logs").mkdir(parents=True)
    (c / "routes").mkdir()
    (c / "logs" / "boot_0007.txt").write_text(BOOT_A)
    (c / "logs" / "boot_0008.txt").write_text(BOOT_B)
    return c


@pytest.fixture
def log_dir(tmp_path):
    return tmp_path / "logs"


# --- pure helpers -------------------------------------------------------------------


def test_split_boots_on_logger_marker():
    chunks = il.split_boots(BOOT_A + BOOT_B)
    assert chunks == [BOOT_A, BOOT_B]
    # A stray preamble is kept as its own chunk rather than dropped.
    assert il.split_boots("stray line\n" + BOOT_A) == ["stray line\n", BOOT_A]
    assert il.split_boots("") == []


def test_summarize_reads_the_identifying_lines():
    s = il.summarize(BOOT_A)
    assert (s.program, s.route, s.runs, s.errors, s.warnings) == ("autonomous", "threepin", 1, 0, 1)
    assert s.lines == 8 and s.duration_s == pytest.approx(12.4)
    assert s.streams == ["health"]
    t = il.summarize(BOOT_B)
    assert (t.program, t.route, t.runs, t.errors, t.streams) == ("opcontrol", "?", 0, 1, ["lift"])
    assert "route=threepin" in str(s) and "[health]" in str(s)


def test_find_card_logs_orders_boots_then_legacy(card):
    (card / "robot_log_19700101_000000.txt").write_text(BOOT_A + BOOT_B)
    (card / "logs" / "boot_0010.txt").write_text(BOOT_B)
    (card / "logs" / "notes.txt").write_text("not a log")
    found = il.find_card_logs(card)
    assert [(c.kind, c.boot) for c in found] == [("boot", 7), ("boot", 8), ("boot", 10), ("legacy", None)]
    assert found[-1].path.name == "robot_log_19700101_000000.txt"


# --- importing ---------------------------------------------------------------------


def test_import_names_by_date_and_boot_and_is_idempotent(card, log_dir):
    imported, skipped, _ = il.import_new(card, log_dir, "09112026")
    assert [i.dest.name for i in imported] == ["09112026-7.txt", "09112026-8.txt"]
    assert [i.label for i in imported] == ["boot 7", "boot 8"]
    assert skipped == []
    assert (log_dir / "09112026-7.txt").read_text() == BOOT_A
    registry = json.loads((log_dir / ".imported.json").read_text())
    assert registry["09112026-7.txt"]["source"] == "logs/boot_0007.txt"

    # Second run: nothing new, card untouched.
    imported, skipped, _ = il.import_new(card, log_dir, "09122026")
    assert imported == [] and len(skipped) == 2
    assert sorted(p.name for p in (card / "logs").iterdir()) == ["boot_0007.txt", "boot_0008.txt"]


def test_import_splits_legacy_dumps(card, log_dir):
    # A legacy dump: two boots appended to one file (different content from
    # the boot files, otherwise the hash dedupe would rightly skip them).
    old_a = BOOT_A.replace("/usd/logs/boot_0007.txt", "/usd/robot_log_19700101_000000.txt")
    old_b = BOOT_B.replace("/usd/logs/boot_0008.txt", "/usd/robot_log_19700101_000000.txt")
    (card / "robot_log_19700101_000000.txt").write_text(old_a + old_b)
    imported, _, _ = il.import_new(card, log_dir, "09112026")
    names = [i.dest.name for i in imported]
    assert names == ["09112026-7.txt", "09112026-8.txt", "09112026-legacy1.txt", "09112026-legacy2.txt"]
    assert [i.label for i in imported[2:]] == ["legacy 1", "legacy 2"]
    assert (log_dir / "09112026-legacy2.txt").read_text() == old_b
    # Re-import skips the legacy chunks too, without consuming legacy numbers.
    imported, _, _ = il.import_new(card, log_dir, "09112026")
    assert imported == []
    (card / "logs" / "boot_0009.txt").write_text(BOOT_B.replace("boot_0008", "boot_0009"))
    (card / "robot_log_2.txt").write_text(BOOT_A.replace("boot_0007", "boot_x"))
    imported, _, _ = il.import_new(card, log_dir, "09112026")
    assert [i.dest.name for i in imported] == ["09112026-9.txt", "09112026-legacy3.txt"]


def test_import_never_overwrites_a_different_file(card, log_dir):
    log_dir.mkdir()
    (log_dir / "09112026-7.txt").write_text("an older, different log with the same name\n")
    imported, _, _ = il.import_new(card, log_dir, "09112026")
    assert [i.dest.name for i in imported] == ["09112026-7b.txt", "09112026-8.txt"]
    assert (log_dir / "09112026-7.txt").read_text().startswith("an older")


def test_dry_run_changes_nothing(card, log_dir):
    imported, _, _ = il.import_new(card, log_dir, "09112026", dry_run=True)
    assert len(imported) == 2
    assert not log_dir.exists() or not any(log_dir.iterdir())


def test_archive_and_delete_only_touch_imported_files(card, log_dir):
    imported, _, _ = il.import_new(card, log_dir, "09112026", archive=True)
    assert len(imported) == 2
    assert sorted(p.name for p in (card / "logs" / "imported").iterdir()) == ["boot_0007.txt", "boot_0008.txt"]
    assert not (card / "logs" / "boot_0007.txt").exists()
    # Already-imported files are left alone even with --delete.
    (card / "logs" / "boot_0007.txt").write_text(BOOT_A)
    imported, skipped, _ = il.import_new(card, log_dir, "09112026", delete=True)
    assert imported == [] and len(skipped) == 1
    assert (card / "logs" / "boot_0007.txt").exists()
    (card / "logs" / "boot_0011.txt").write_text(BOOT_B.replace("boot_0008", "boot_0011"))
    imported, _, _ = il.import_new(card, log_dir, "09112026", delete=True)
    assert [i.dest.name for i in imported] == ["09112026-11.txt"]
    assert not (card / "logs" / "boot_0011.txt").exists()


# --- damaged cards -----------------------------------------------------------------


@pytest.fixture
def break_read(monkeypatch):
    """Make one card file raise EIO, the way a bad FAT cluster chain does."""

    def _break(target: Path) -> None:
        real_read_text, real_open = Path.read_text, Path.open

        def fail(self, *args, **kwargs):
            if self == target:
                raise OSError(errno.EIO, "Input/output error", str(self))
            return None

        def read_text(self, *args, **kwargs):
            fail(self)
            return real_read_text(self, *args, **kwargs)

        def open_(self, *args, **kwargs):
            fail(self)
            return real_open(self, *args, **kwargs)

        monkeypatch.setattr(Path, "read_text", read_text)
        monkeypatch.setattr(Path, "open", open_)

    return _break


def test_unreadable_file_does_not_strand_the_boots_after_it(card, log_dir, break_read):
    # boot 7 is corrupt; boot 8 sits after it in the sort order and must still land.
    break_read(card / "logs" / "boot_0007.txt")
    imported, skipped, unreadable = il.import_new(card, log_dir, "09112026")
    assert [i.dest.name for i in imported] == ["09112026-8.txt"]
    assert skipped == []
    assert [u.source.path.name for u in unreadable] == ["boot_0007.txt"]
    assert unreadable[0].error.errno == errno.EIO
    # A corrupt file is not recorded as imported, so a later run retries it.
    registry = json.loads((log_dir / ".imported.json").read_text())
    assert list(registry) == ["09112026-8.txt"]


def test_cli_warns_about_unreadable_files_and_exits_nonzero(card, log_dir, break_read, capsys):
    break_read(card / "logs" / "boot_0007.txt")
    assert il.main(["--card", str(card), "--log-dir", str(log_dir), "--date", "09112026"]) == 1
    captured = capsys.readouterr()
    assert "09112026-8.txt" in captured.out  # the healthy boot still imported
    assert "boot_0007.txt" in captured.err and "Input/output error" in captured.err


def test_readable_prefix_size_reports_what_the_card_serves(card, tmp_path):
    healthy = card / "logs" / "boot_0007.txt"
    assert il.readable_prefix_size(healthy) == healthy.stat().st_size
    assert il.readable_prefix_size(tmp_path) == 0  # unopenable: nothing salvageable


def test_cli_reports_and_lists(card, log_dir, capsys):
    assert il.main(["--card", str(card), "--log-dir", str(log_dir), "--date", "09112026"]) == 0
    out = capsys.readouterr().out
    assert "imported 2 boot(s)" in out and "09112026-7.txt" in out and "route=threepin" in out
    assert "visualize.py dashboard 09112026-7.txt" in out
    assert il.main(["--card", str(card), "--log-dir", str(log_dir), "--list"]) == 0
    out = capsys.readouterr().out
    assert "logs/boot_0007.txt" in out and "09112026-7.txt" in out
    assert il.main(["--card", str(card), "--log-dir", str(log_dir)]) == 0
    assert "nothing new" in capsys.readouterr().out


def test_cli_refuses_missing_card(tmp_path, capsys):
    assert il.main(["--card", str(tmp_path / "nope")]) == 2
    assert "nothing mounted" in capsys.readouterr().err
