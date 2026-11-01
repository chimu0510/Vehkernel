# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for check_examples.py (spec 0013 Seam A)."""

from pathlib import Path
import sys
import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT / "tools" / "generators") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools" / "generators"))
if str(REPO_ROOT / "tools" / "examples") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools" / "examples"))

from check_examples import LESSON_SECTIONS, check_examples, main  # noqa: E402

C50C_BOARD_TOML = REPO_ROOT / "boards" / "c50c-v1_1" / "board.toml"
C50C_EXAMPLES_TOML = REPO_ROOT / "examples" / "c50c-v1_1" / "examples.toml"
C50C_EXAMPLES_DIR = REPO_ROOT / "examples" / "c50c-v1_1"
SOC_ROOT = REPO_ROOT / "soc"


def create_minimal_lesson(path: Path, skip_section: str | None = None) -> None:
    lines = ["# Minimal Example Lesson\n"]
    for sec in LESSON_SECTIONS:
        if sec == skip_section:
            continue
        lines.append(f"## {sec}\nContent for {sec}\n")
    path.write_text("\n".join(lines), encoding="utf-8")


def create_mock_example_tree(tmp_path: Path, name: str = "01-led", skip_section: str | None = None) -> Path:
    ex_dir = tmp_path / name
    ex_dir.mkdir(parents=True, exist_ok=True)
    (ex_dir / "main.cpp").write_text("// test main\nint main() {}\n", encoding="utf-8")
    create_minimal_lesson(ex_dir / "README.md", skip_section=skip_section)
    return tmp_path


def test_check_examples_c50c_passes():
    """Verify that the repository's C50C V1.1 examples.toml passes with zero errors."""
    errors = check_examples(
        examples_toml=C50C_EXAMPLES_TOML,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert errors == [], f"Unexpected errors on C50C V1.1: {errors}"


def test_unknown_board_component(tmp_path: Path):
    """Fails when an unknown board component is named in introduces."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        'introduces = ["pin:LIGHT_R", "pin:LIGHT_G", "pin:LIGHT_B", "flash:FIRMWARE"]',
        'introduces = ["pin:LIGHT_R", "pin:LIGHT_G", "pin:LIGHT_B", "flash:FIRMWARE", "device:NONEXISTENT"]',
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("unknown board component 'device:NONEXISTENT'" in e for e in errors)


def test_board_component_neither_introduced_nor_not_covered(tmp_path: Path):
    """Fails when a board component is omitted from introduces, not_covered, and pending."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        '"pin:REV_ID0" = "revision-ID pin; read at boot for board verification (ADR-0010)"\n',
        "",
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("board component 'pin:REV_ID0' neither introduced nor listed as not covered" in e for e in errors)


def test_board_component_both_introduced_and_not_covered(tmp_path: Path):
    """Fails when a board component is both introduced and listed in not_covered."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        'introduces = ["pin:LIGHT_R", "pin:LIGHT_G", "pin:LIGHT_B", "flash:FIRMWARE"]',
        'introduces = ["pin:LIGHT_R", "pin:LIGHT_G", "pin:LIGHT_B", "flash:FIRMWARE", "pin:REV_ID0"]',
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("both introduced by '01-led' and listed in [not_covered]" in e for e in errors)


def test_component_introduced_twice(tmp_path: Path):
    """Fails when two examples introduce the same component."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "01-led")
    create_mock_example_tree(ex_tree, "02-user-key")

    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    # Replace pending user_key and add second example that re-introduces pin:LIGHT_R
    toml_text = base_toml.replace('"user_key" = "02-user-key"\n', "")
    toml_text += """
[[example]]
name = "02-user-key"
summary = "User key example"
introduces = ["user_key", "pin:LIGHT_R"]
uses = ["pin:LIGHT_R"]
sitl = false
"""
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(toml_text, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
    )
    assert any("component 'pin:LIGHT_R' introduced twice" in e for e in errors)


def test_component_used_before_introduced(tmp_path: Path):
    """Fails when an example uses a component before it is introduced."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        "uses = []",
        'uses = ["device:IMU0"]',
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("component 'device:IMU0' used by '01-led' before it is introduced" in e for e in errors)


def test_undeclared_firmware_component(tmp_path: Path):
    """Fails when an undeclared firmware component is used."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        "uses = []",
        'uses = ["undeclared_fw_thing"]',
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("undeclared firmware component 'undeclared_fw_thing' used by '01-led'" in e for e in errors)


def test_undeclared_firmware_component_introduced(tmp_path: Path):
    """Fails when an undeclared firmware component is introduced."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        'introduces = ["pin:LIGHT_R", "pin:LIGHT_G", "pin:LIGHT_B", "flash:FIRMWARE"]',
        'introduces = ["pin:LIGHT_R", "pin:LIGHT_G", "pin:LIGHT_B", "flash:FIRMWARE", "undeclared_fw_thing"]',
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("undeclared firmware component 'undeclared_fw_thing' introduced by '01-led'" in e for e in errors)


def test_unknown_board_component_used(tmp_path: Path):
    """Fails when a prefixed board component not in the board definition is used."""
    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace(
        "uses = []",
        'uses = ["device:NONEXISTENT"]',
    )
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=C50C_EXAMPLES_DIR,
        soc_root=SOC_ROOT,
    )
    assert any("unknown board component 'device:NONEXISTENT' used by '01-led'" in e for e in errors)


def test_gap_in_numbering(tmp_path: Path):
    """Fails when example naming has a gap in numbering."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "02-led")

    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    bad_toml = base_toml.replace('name = "01-led"', 'name = "02-led"')
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(bad_toml, encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
    )
    assert any("gap in numbering: example at index 0 has name '02-led', expected prefix '01-'" in e for e in errors)


def test_directory_not_listed(tmp_path: Path):
    """Fails when a directory exists on disk but is not listed in examples.toml."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "01-led")
    (ex_tree / "02-unlisted").mkdir()

    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(C50C_EXAMPLES_TOML.read_text(encoding="utf-8"), encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
    )
    assert any("directory '02-unlisted' exists on disk but is not listed in examples.toml" in e for e in errors)


def test_directory_missing_main_cpp(tmp_path: Path):
    """Fails when an example directory is missing main.cpp."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "01-led")
    (ex_tree / "01-led" / "main.cpp").unlink()

    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(C50C_EXAMPLES_TOML.read_text(encoding="utf-8"), encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
    )
    assert any("example '01-led' missing main.cpp" in e for e in errors)


def test_directory_missing_readme(tmp_path: Path):
    """Fails when an example directory is missing README.md."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "01-led")
    (ex_tree / "01-led" / "README.md").unlink()

    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(C50C_EXAMPLES_TOML.read_text(encoding="utf-8"), encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
    )
    assert any("example '01-led' missing README.md" in e for e in errors)


def test_lesson_missing_section(tmp_path: Path):
    """Fails when a lesson README.md is missing a required section."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "01-led", skip_section="Exercises")

    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(C50C_EXAMPLES_TOML.read_text(encoding="utf-8"), encoding="utf-8")

    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
    )
    assert any("example '01-led' lesson missing section 'Exercises'" in e for e in errors)


def test_sitl_target_mismatch(tmp_path: Path):
    """Fails when examples declaring sitl = true do not match sitl_targets."""
    ex_tree = tmp_path / "examples"
    create_mock_example_tree(ex_tree, "01-led")

    base_toml = C50C_EXAMPLES_TOML.read_text(encoding="utf-8")
    sitl_toml = base_toml.replace("sitl = false", "sitl = true")
    toml_path = tmp_path / "examples.toml"
    toml_path.write_text(sitl_toml, encoding="utf-8")

    # sitl_targets is empty, but 01-led declared sitl = true
    errors = check_examples(
        examples_toml=toml_path,
        board_toml=C50C_BOARD_TOML,
        examples_dir=ex_tree,
        soc_root=SOC_ROOT,
        sitl_targets=[],
    )
    assert any("sitl-examples targets mismatch" in e for e in errors)


def test_cli_invocation():
    """Verify main() entry point with required flag arguments."""
    rc = main([
        "--examples-toml",
        str(C50C_EXAMPLES_TOML),
        "--board-toml",
        str(C50C_BOARD_TOML),
        "--examples-dir",
        str(C50C_EXAMPLES_DIR),
        "--soc-root",
        str(SOC_ROOT),
    ])
    assert rc == 0
