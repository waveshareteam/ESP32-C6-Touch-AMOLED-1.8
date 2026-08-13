"""Synthetic contract tests for the exact example-CI selector."""

from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import discover_examples as discovery  # noqa: E402


class DiscoveryRoutingTests(unittest.TestCase):
    def selected(self, surface: str, *paths: str) -> list[dict[str, str]]:
        return discovery.route_examples(surface, list(paths))

    def test_full_matrix_has_expected_entries(self) -> None:
        self.assertEqual(7, len(discovery.all_examples("esp-idf")))
        self.assertEqual(23, len(discovery.all_examples("arduino")))
        class Args:
            surface = "esp-idf"
            idf_versions = "v5.5.5,v6.0.2"
            arduino_core = "3.3.11"
            fqbn = "test"
        self.assertEqual(14, len(discovery.build_matrix(Args(), discovery.all_examples("esp-idf"))["include"]))
        self.assertEqual(37, 14 + len(discovery.all_examples("arduino")))

    def test_relative_component_override_paths_exist(self) -> None:
        """Every relative Component Manager override stays within this checkout."""
        override_path = re.compile(r"^\s*override_path:\s*[\"']?([^\"'#\s]+)")
        missing: list[str] = []
        for example in discovery.list_esp_idf_examples():
            project = ROOT / example["path"]
            for manifest in project.rglob("idf_component.yml"):
                for line_number, line in enumerate(manifest.read_text(encoding="utf-8").splitlines(), 1):
                    match = override_path.match(line)
                    if match:
                        value = Path(match.group(1))
                        target = (manifest.parent / value).resolve()
                        if value.is_absolute() or not target.is_relative_to(ROOT) or not target.exists():
                            missing.append(f"{manifest.relative_to(ROOT)}:{line_number}: {value}")
        self.assertEqual([], missing, "invalid relative override_path targets:\n" + "\n".join(missing))

    def test_test_app_component_paths_are_compatible_and_fail_fast(self) -> None:
        cmake = (ROOT / "examples/esp-idf/03_esp-brookesia/components/brookesia_core/test_apps/CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('set(test_app_components "$ENV{IDF_PATH}/tools/test_apps/components")', cmake)
        self.assertIn('set(legacy_test_app_components "$ENV{IDF_PATH}/tools/unit-test-app/components")', cmake)
        self.assertIn('if(EXISTS "${test_app_components}")', cmake)
        self.assertIn('elseif(EXISTS "${legacy_test_app_components}")', cmake)
        self.assertIn('message(FATAL_ERROR', cmake)

    def test_root_markdown_selects_no_builds(self) -> None:
        self.assertFalse(self.selected("esp-idf", "README.md"))
        self.assertFalse(self.selected("arduino", "README.md"))

    def test_idf_readme_selects_no_builds(self) -> None:
        self.assertFalse(self.selected("esp-idf", "examples/esp-idf/01_AXP2101/README.md"))

    def test_arduino_sketch_readme_selects_no_builds(self) -> None:
        self.assertFalse(self.selected("arduino", "examples/arduino/examples/01_HelloWorld/README.md"))

    def test_bundled_library_markdown_selects_no_builds(self) -> None:
        self.assertFalse(self.selected("arduino", "examples/arduino/libraries/lvgl/README.md"))

    def test_direct_idf_source_selects_only_its_project(self) -> None:
        selected = self.selected("esp-idf", "examples/esp-idf/04_QMI8658/main/main.c")
        self.assertEqual(["examples/esp-idf/04_QMI8658"], [item["path"] for item in selected])

    def test_idf_cmake_and_direct_config_select_their_project(self) -> None:
        for path in (
            "examples/esp-idf/04_QMI8658/CMakeLists.txt",
            "examples/esp-idf/04_QMI8658/sdkconfig.defaults",
        ):
            self.assertEqual(["examples/esp-idf/04_QMI8658"], [item["path"] for item in self.selected("esp-idf", path)])

    def test_direct_arduino_source_selects_only_its_sketch(self) -> None:
        selected = self.selected("arduino", "examples/arduino/examples/01_HelloWorld/01_HelloWorld.ino")
        self.assertEqual(["examples/arduino/examples/01_HelloWorld"], [item["path"] for item in selected])

    def test_shared_arduino_library_source_selects_all_sketches(self) -> None:
        self.assertEqual(23, len(self.selected("arduino", "examples/arduino/libraries/lvgl/src/lvgl.c")))

    def test_workflow_input_selects_full_surface(self) -> None:
        self.assertEqual(7, len(self.selected("esp-idf", ".github/workflows/examples.yml")))
        self.assertEqual(23, len(self.selected("arduino", ".github/workflows/examples.yml")))

    def test_firmware_kinds_never_enter_examples_matrix(self) -> None:
        for path in ("Firmware/README.md", "Firmware/source/main.c", "Firmware/image.bin", "Firmware/delivery.zip"):
            self.assertFalse(self.selected("esp-idf", path), path)
            self.assertFalse(self.selected("arduino", path), path)

    def test_scope_reports_firmware_and_unknown_paths(self) -> None:
        scope = discovery.route_scope(["Firmware/image.bin"])
        self.assertTrue(scope["firmware_touched"])
        self.assertEqual([], scope["unknown_paths"])
        scope = discovery.route_scope(["tools/new_input.dat"])
        discovery.route_examples("esp-idf", ["tools/new_input.dat"], scope)
        self.assertEqual(["tools/new_input.dat"], scope["unknown_paths"])

    def test_config_selects_only_the_idf_full_surface(self) -> None:
        self.assertFalse(self.selected("esp-idf", "config/README.md"))
        self.assertFalse(self.selected("arduino", "config/README.md"))
        self.assertEqual(7, len(self.selected("esp-idf", "config/ci.defaults")))
        self.assertFalse(self.selected("arduino", "config/ci.defaults"))

    def test_governance_and_checker_changes_remain_lightweight(self) -> None:
        for path in (
            "LICENSE",
            ".github/ISSUE_TEMPLATE/bug_report.yml",
            "scripts/audit_markdown.py",
            "tests/test_audit_markdown.py",
            "assets/ESP32-C6-Touch-AMOLED-1.8.jpg",
        ):
            self.assertFalse(self.selected("esp-idf", path), path)
            self.assertFalse(self.selected("arduino", path), path)

    def test_rename_and_deletion_old_paths_are_routed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / "changed.txt"
            changed.write_text("R100\texamples/esp-idf/04_QMI8658/main/main.c\tdocs/moved.md\nD\texamples/arduino/examples/01_HelloWorld/01_HelloWorld.ino\n", encoding="utf-8")
            paths = discovery.changed_paths(None, "HEAD", str(changed))
        self.assertEqual(1, len(self.selected("esp-idf", *paths)))
        self.assertEqual(1, len(self.selected("arduino", *paths)))

    def test_unknown_complete_path_is_conservative(self) -> None:
        self.assertEqual(7, len(self.selected("esp-idf", "tools/new_input.dat")))
        self.assertEqual(23, len(self.selected("arduino", "tools/new_input.dat")))

    def test_routing_input_is_conservative_and_visible_in_scope(self) -> None:
        paths = ["tests/test_discover_examples.py"]
        scope = discovery.route_scope(paths)
        self.assertEqual(7, len(discovery.route_examples("esp-idf", paths, scope)))
        self.assertEqual(paths, scope["unknown_paths"])
        scope = discovery.route_scope(paths)
        self.assertEqual(23, len(discovery.route_examples("arduino", paths, scope)))
        self.assertEqual(paths, scope["unknown_paths"])

    def test_empty_or_unavailable_diff_fails_closed(self) -> None:
        with self.assertRaises(discovery.ScopeUnavailable):
            discovery.changed_paths(None, "HEAD", None)
        with tempfile.TemporaryDirectory() as directory:
            empty = Path(directory) / "empty.txt"
            empty.write_text("", encoding="utf-8")
            with self.assertRaises(discovery.ScopeUnavailable):
                discovery.changed_paths(None, "HEAD", str(empty))

    def test_manual_selectors_accept_all_name_and_path(self) -> None:
        examples = discovery.all_examples("esp-idf")
        self.assertEqual(7, len([item for item in examples if discovery.matches_selector(item, "all")]))
        self.assertEqual(1, len([item for item in examples if discovery.matches_selector(item, "04_QMI8658")]))
        self.assertEqual(1, len([item for item in examples if discovery.matches_selector(item, "examples/esp-idf/04_QMI8658")]))

    def test_workflow_cli_flags_and_github_output_format(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "github-output.txt"
            result = subprocess.run([
                sys.executable, "scripts/discover_examples.py", "--surface", "esp-idf",
                "--selector", "all", "--base-ref", "", "--head-ref", "HEAD",
                "--idf-versions", "v5.5.5,v6.0.2", "--github-output", str(output),
            ], cwd=ROOT, check=True, text=True, capture_output=True)
            self.assertEqual(14, len(json.loads(result.stdout)["matrix"]["include"]))
            self.assertIn("matrix={\"include\":", output.read_text(encoding="utf-8"))
            self.assertIn("count=14", output.read_text(encoding="utf-8"))
            self.assertIn("scope={\"firmware_touched\":false", output.read_text(encoding="utf-8"))

    def test_pr_empty_selector_changed_files_routes_docs_and_source(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / "changed.txt"
            output = Path(directory) / "github-output.txt"
            changed.write_text("README.md\n", encoding="utf-8")
            command = [
                sys.executable, "scripts/discover_examples.py", "--surface", "esp-idf",
                "--selector", "", "--base-ref", "base", "--head-ref", "head",
                "--changed-files", str(changed), "--idf-versions", "v5.5.5,v6.0.2",
                "--github-output", str(output),
            ]
            docs = subprocess.run(command, cwd=ROOT, check=True, text=True, capture_output=True)
            self.assertEqual(0, len(json.loads(docs.stdout)["matrix"]["include"]))
            changed.write_text("examples/esp-idf/04_QMI8658/main/main.c\n", encoding="utf-8")
            source = subprocess.run(command, cwd=ROOT, check=True, text=True, capture_output=True)
            self.assertEqual(2, len(json.loads(source.stdout)["matrix"]["include"]))


if __name__ == "__main__":
    unittest.main()
