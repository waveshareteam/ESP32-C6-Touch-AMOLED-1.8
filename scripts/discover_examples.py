#!/usr/bin/env python3
"""Fail-closed discovery and changed-file routing for example CI matrices."""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import re
import subprocess
from pathlib import Path


ESP_IDF_ROOT = Path("examples/esp-idf")
ARDUINO_ROOTS = (Path("examples/arduino"), Path("examples/arduino-v2"))
GLOBAL_PATTERNS = (
    ".github/workflows/**",
    "scripts/discover_examples.py",
    "releases/package_firmware.py",
)
NON_BUILD_PATTERNS = (
    ".github/ISSUE_TEMPLATE/**",
    ".github/PULL_REQUEST_TEMPLATE*",
    "CODE_OF_CONDUCT*",
    "CONTRIBUTING*",
    "LICENSE*",
    "SECURITY*",
    "SUPPORT*",
    "THIRD_PARTY*",
    "assets/markdown-audit-config.json",
    "config/markdown-audit-config.json",
    "scripts/audit_markdown.py",
    "tests/**",
)
DOCUMENTATION_ASSET_PATTERNS = (
    "assets/*.gif", "assets/*.jpeg", "assets/*.jpg", "assets/*.png", "assets/*.svg", "assets/*.webp",
    "docs/**/*.gif", "docs/**/*.jpeg", "docs/**/*.jpg", "docs/**/*.png", "docs/**/*.svg", "docs/**/*.webp",
)


class ScopeUnavailable(RuntimeError):
    """The event did not provide a complete, reviewable change scope."""


def sanitize_name(path: Path) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "-", path.as_posix()).strip(".-")


def is_esp_idf_project(path: Path) -> bool:
    return (path / "CMakeLists.txt").is_file() and (path / "main").is_dir()


def list_esp_idf_examples() -> list[dict[str, str]]:
    if not ESP_IDF_ROOT.is_dir():
        return []
    return [
        {"name": sanitize_name(path.relative_to(ESP_IDF_ROOT)), "path": path.as_posix()}
        for path in sorted(ESP_IDF_ROOT.iterdir(), key=lambda item: item.as_posix().lower())
        if path.is_dir() and is_esp_idf_project(path)
    ]


def list_arduino_examples() -> list[dict[str, str]]:
    examples: list[dict[str, str]] = []
    for root in ARDUINO_ROOTS:
        sketches_root, libraries = root / "examples", root / "libraries"
        if not sketches_root.is_dir():
            continue
        seen: set[Path] = set()
        for sketch in sorted(sketches_root.rglob("*.ino"), key=lambda item: item.as_posix().lower()):
            project = sketch.parent
            if project in seen:
                continue
            seen.add(project)
            examples.append({
                "name": sanitize_name(Path(root.name) / project.relative_to(sketches_root)),
                "path": project.as_posix(),
                "libraries": libraries.as_posix(),
            })
    return examples


def all_examples(surface: str) -> list[dict[str, str]]:
    return list_esp_idf_examples() if surface == "esp-idf" else list_arduino_examples()


def matches_selector(example: dict[str, str], selector: str) -> bool:
    selector = selector.replace("\\", "/").strip().strip("/")
    if selector == "all":
        return True
    path = example["path"].strip("/")
    return selector in {path, example["name"], Path(path).name} or path.startswith(selector + "/")


def is_documentation(path: str) -> bool:
    return path.lower().endswith((".md", ".markdown")) or any(
        fnmatch.fnmatch(path, pattern) for pattern in DOCUMENTATION_ASSET_PATTERNS
    )


def is_firmware_path(path: str) -> bool:
    return path.casefold().startswith("firmware/")


def changed_paths(base_ref: str | None, head_ref: str, changed_file: str | None) -> list[str]:
    if changed_file:
        try:
            lines = Path(changed_file).read_text(encoding="utf-8").splitlines()
        except OSError as error:
            raise ScopeUnavailable(f"cannot read changed-files input: {error}") from error
        paths: list[str] = []
        for line in lines:
            fields = line.split("\t")
            if not line.strip():
                continue
            # Accept plain paths and Git name-status records, including renames.
            paths.extend(field.strip() for field in (fields[1:] if len(fields) > 1 else fields) if field.strip())
        if not paths:
            raise ScopeUnavailable("changed-files input is empty")
        return paths
    if not base_ref or set(base_ref) == {"0"}:
        raise ScopeUnavailable("a non-empty base ref is required for automatic routing")
    try:
        result = subprocess.run(
            ["git", "diff", "--name-status", "-M", f"{base_ref}...{head_ref}"],
            check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
    except subprocess.CalledProcessError as error:
        raise ScopeUnavailable(f"cannot read diff: {error.stderr.strip()}") from error
    paths: list[str] = []
    for line in result.stdout.splitlines():
        fields = line.split("\t")
        if len(fields) < 2:
            continue
        # Renames carry both old and new paths; deletions carry the removed path.
        paths.extend(field.strip() for field in fields[1:] if field.strip())
    if not paths:
        raise ScopeUnavailable("diff is empty")
    return paths


def route_scope(paths: list[str]) -> dict[str, object]:
    """Expose non-build surfaces so CI can report their independent scope."""
    return {
        "firmware_touched": any(is_firmware_path(path.replace("\\", "/").strip("/")) for path in paths),
        "unknown_paths": [],
    }


def route_examples(surface: str, paths: list[str], scope: dict[str, object] | None = None) -> list[dict[str, str]]:
    examples = all_examples(surface)
    selected: list[dict[str, str]] = []
    global_change = False
    unknown_change = False
    for raw_path in paths:
        path = raw_path.replace("\\", "/").strip("/")
        if not path or is_firmware_path(path):
            continue
        if is_documentation(path) or any(fnmatch.fnmatch(path, pattern) for pattern in NON_BUILD_PATTERNS):
            continue
        if any(fnmatch.fnmatch(path, pattern) for pattern in GLOBAL_PATTERNS):
            global_change = True
            continue
        if path.startswith("config/"):
            if surface == "esp-idf":
                global_change = True
            continue
        if surface == "esp-idf" and any(path.startswith(root.as_posix() + "/") for root in ARDUINO_ROOTS):
            continue
        if surface == "arduino" and path.startswith(ESP_IDF_ROOT.as_posix() + "/"):
            continue
        matched = False
        for example in examples:
            example_path = example["path"].strip("/")
            if path == example_path or path.startswith(example_path + "/"):
                selected.append(example)
                matched = True
            if surface == "arduino":
                libraries = example["libraries"].strip("/")
                if path == libraries or path.startswith(libraries + "/"):
                    global_change = True
                    matched = True
        if not matched:
            unknown_change = True
            if scope is not None:
                cast_unknown = scope["unknown_paths"]
                assert isinstance(cast_unknown, list)
                cast_unknown.append(path)
    if global_change or unknown_change:
        return examples
    unique = {item["path"]: item for item in selected}
    return [unique[key] for key in sorted(unique)]


def select_examples(args: argparse.Namespace, scope: dict[str, object] | None = None) -> list[dict[str, str]]:
    selector = args.selector.replace("\\", "/").strip().strip("/")
    examples = all_examples(args.surface)
    if selector:
        selected = [example for example in examples if matches_selector(example, selector)]
        if not selected:
            raise ScopeUnavailable(f"selector did not match a {args.surface} example: {selector}")
        return selected
    paths = changed_paths(args.base_ref, args.head_ref, args.changed_files)
    if scope is not None:
        scope.update(route_scope(paths))
    return route_examples(args.surface, paths, scope)


def build_matrix(args: argparse.Namespace, selected: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    include: list[dict[str, str]] = []
    if args.surface == "esp-idf":
        for example in selected:
            for idf in (item.strip() for item in args.idf_versions.split(",") if item.strip()):
                include.append({"name": example["name"], "path": example["path"], "idf": idf})
    else:
        include = [{**example, "core": args.arduino_core, "fqbn": args.fqbn} for example in selected]
    return {"include": include}


def write_github_output(path: str, matrix: dict[str, list[dict[str, str]]], scope: dict[str, object]) -> None:
    if not path:
        return
    with open(path, "a", encoding="utf-8") as output:
        output.write(f"matrix={json.dumps(matrix, separators=(',', ':'))}\n")
        output.write(f"count={len(matrix['include'])}\n")
        output.write(f"scope={json.dumps(scope, separators=(',', ':'))}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--surface", choices=("esp-idf", "arduino"), required=True)
    parser.add_argument("--selector", default="")
    parser.add_argument("--base-ref")
    parser.add_argument("--head-ref", default="HEAD")
    parser.add_argument("--changed-files")
    parser.add_argument("--idf-versions", default="v5.5.5,v6.0.2")
    parser.add_argument("--arduino-core", default="3.3.11")
    parser.add_argument("--fqbn", default="esp32:esp32:esp32c6:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB")
    parser.add_argument("--github-output", default=os.environ.get("GITHUB_OUTPUT", ""))
    args = parser.parse_args()
    try:
        scope: dict[str, object] = {"firmware_touched": False, "unknown_paths": []}
        matrix = build_matrix(args, select_examples(args, scope))
    except ScopeUnavailable as error:
        parser.error(str(error))
        return 2
    write_github_output(args.github_output, matrix, scope)
    print(json.dumps({"matrix": matrix, "scope": scope}, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
