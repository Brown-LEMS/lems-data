#!/usr/bin/env python3
"""Build the static lems-data Doxygen site without building C++."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ENV_NAMES = ("LEMS_DOCS_OUTPUT", "LEMS_DATA_DOCS_OUTPUT")
DOXYGEN_ENV_NAMES = ("DOXYGEN", "LEMS_DOXYGEN")
OUTPUT_MARKER = ".lems-docs-generated"
GENERATED_OUTPUT_NAMES = ("html", "xml", ".doxygen", "doxygen-warnings.log")


def default_output(root: Path = ROOT) -> Path:
    """Return an output directory outside the checkout."""

    for name in OUTPUT_ENV_NAMES:
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()
    return root.parent / "lems-data-docs-site"


def _configured_doxygen() -> str | None:
    for name in DOXYGEN_ENV_NAMES:
        value = os.environ.get(name)
        if value:
            return value
    return shutil.which("doxygen")


def _quote_config_path(path: Path) -> str:
    # Doxygen accepts quoted paths and uses backslash as an escape character.
    return '"' + str(path).replace("\\", "/").replace('"', '\\"') + '"'


def _config_list(paths: Iterable[Path]) -> str:
    return " ".join(_quote_config_path(path) for path in paths)


def _replace_template(template: str, values: dict[str, str]) -> str:
    rendered = template
    for key, value in values.items():
        rendered = rendered.replace(f"@{key}@", value)
    required = (
        "@OUTPUT_DIRECTORY@",
        "@PROJECT_ROOT@",
        "@INPUT@",
        "@MAINPAGE@",
        "@WARN_LOGFILE@",
    )
    unresolved = sorted(token for token in required if token in rendered)
    if unresolved:
        raise RuntimeError("unresolved Doxygen template placeholders: " + ", ".join(unresolved))
    return rendered


def _asset_version(path: Path) -> str:
    """Return a short stable content hash for a generated theme asset."""

    return hashlib.sha256(path.read_bytes()).hexdigest()[:12]


def _postprocess_html(html_root: Path, include_script: bool, source_root: Path) -> int:
    """Include local theme assets and stage links to migration helpers."""

    source_link = re.compile(r'href="\.\./(migration/[^"#?]+)"')
    css_source = source_root / "docs" / "site" / "assets" / "lems-docs.css"
    js_source = source_root / "docs" / "site" / "assets" / "lems-docs.js"
    css_href = f"lems-docs.css?v={_asset_version(css_source)}" if css_source.is_file() else "lems-docs.css"
    js_src = f"lems-docs.js?v={_asset_version(js_source)}" if include_script else "lems-docs.js"
    script_tag = f'<script defer src="{js_src}"></script>'
    script_pattern = re.compile(r'<script defer src="lems-docs\.js(?:\?[^"/]*)?"></script>')
    stylesheet_pattern = re.compile(r'href="lems-docs\.css(?:\?[^"/]*)?"')
    changed = 0
    for page in sorted(html_root.rglob("*.html")):
        try:
            text = page.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        original = text
        if css_source.is_file():
            text = stylesheet_pattern.sub(f'href="{css_href}"', text)
        if include_script and script_tag not in text:
            if script_pattern.search(text):
                text = script_pattern.sub(script_tag, text, count=1)
            elif "</head>" in text:
                text = text.replace("</head>", f"  {script_tag}\n</head>", 1)
            else:
                text = script_tag + "\n" + text

        def stage_source_link(match: re.Match[str]) -> str:
            relative = match.group(1)
            source = source_root / relative
            destination = html_root / relative
            if source.is_file():
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
                return f'href="{relative}"'
            return match.group(0)

        text = source_link.sub(stage_source_link, text)
        if text != original:
            page.write_text(text, encoding="utf-8")
            changed += 1
    return changed


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build the lems-data static Doxygen site without compiling C++"
    )
    parser.add_argument(
        "--doxygen",
        default=_configured_doxygen(),
        help="Doxygen executable (or set DOXYGEN); required when it is not on PATH",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output(),
        help="output directory containing html/, xml/, and warnings log",
    )
    parser.add_argument(
        "--no-xml",
        action="store_true",
        help="skip XML generation (XML is enabled by default for link auditing)",
    )
    parser.add_argument(
        "--keep-config",
        action="store_true",
        help="retain the generated Doxygen config under output/.doxygen",
    )
    return parser.parse_args()


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _validate_output_path(root: Path, output: Path, raw_output: Path | None = None) -> None:
    """Reject output paths whose cleanup could reach the checkout or user data."""

    root = root.resolve()
    output = output.resolve()
    raw_output = raw_output or output
    if raw_output.is_symlink():
        raise RuntimeError(f"refusing symbolic-link documentation output: {raw_output}")
    if output == Path("/"):
        raise RuntimeError("refusing filesystem root as documentation output")
    if output == Path.home():
        raise RuntimeError("refusing the user home directory as documentation output")
    if output == root or _is_relative_to(output, root):
        raise RuntimeError(f"refusing documentation output inside the source repository: {output}")
    if _is_relative_to(root, output):
        raise RuntimeError(f"refusing documentation output that contains the source repository: {output}")


def _marker_source(marker: Path) -> Path | None:
    try:
        text = marker.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"could not read output ownership marker {marker}: {exc}") from exc
    for line in text.splitlines():
        if line.startswith("source="):
            value = line.partition("=")[2].strip()
            if value:
                return Path(value).expanduser().resolve()
    return None


def _refresh_owned_output(output: Path, root: Path = ROOT) -> None:
    """Refresh only a safe output directory previously marked by this build."""

    root = root.resolve()
    output = output.resolve()
    _validate_output_path(root, output)
    if output.exists() and output.is_symlink():
        raise RuntimeError(f"refusing symbolic-link documentation output: {output}")
    if output.exists() and not output.is_dir():
        raise RuntimeError(f"documentation output is not a directory: {output}")

    generated_paths = [output / name for name in GENERATED_OUTPUT_NAMES]
    for generated in generated_paths:
        if generated.is_symlink():
            raise RuntimeError(f"refusing symbolic-link generated output: {generated}")

    marker = output / OUTPUT_MARKER
    if marker.is_symlink():
        raise RuntimeError(f"refusing symbolic-link output ownership marker: {marker}")
    if not marker.is_file():
        # An unmarked html/xml tree may belong to a user or another tool. Do
        # not remove or overwrite it; callers can choose a fresh output path.
        existing = [path for path in generated_paths if path.exists()]
        if existing:
            names = ", ".join(str(path) for path in existing)
            raise RuntimeError(f"refusing unmarked generated output; choose a fresh path: {names}")
        return

    source = _marker_source(marker)
    if source != root:
        raise RuntimeError(
            f"documentation output marker belongs to {source or 'an unknown source tree'}, not {root}"
        )
    for generated in (output / "html", output / "xml"):
        if generated.exists():
            if not generated.is_dir() or generated.is_symlink():
                raise RuntimeError(f"refusing non-directory generated output: {generated}")
            if generated.resolve().parent != output:
                raise RuntimeError(f"refusing generated output outside its owner: {generated}")
            shutil.rmtree(generated)
    warnings = output / "doxygen-warnings.log"
    if warnings.exists():
        if warnings.is_symlink() or not warnings.is_file():
            raise RuntimeError(f"refusing non-regular warning log: {warnings}")
        warnings.unlink()


def _warning_count(path: Path) -> int:
    if not path.is_file():
        return 0
    try:
        return sum(1 for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip())
    except OSError:
        return 0


def main() -> int:
    args = _parse_args()
    if not args.doxygen:
        print(
            "error: Doxygen is required. Supply --doxygen PATH or set DOXYGEN "
            "to the official binary; no package installation is attempted.",
            file=sys.stderr,
        )
        return 2

    doxygen = Path(args.doxygen).expanduser()
    if not doxygen.exists() and shutil.which(str(doxygen)) is None:
        print(f"error: Doxygen executable was not found: {args.doxygen}", file=sys.stderr)
        return 2
    resolved_doxygen = shutil.which(str(doxygen)) or str(doxygen)

    root = ROOT.resolve()
    raw_output = args.output.expanduser()
    output = raw_output.resolve()
    try:
        _validate_output_path(root, output, raw_output)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    html_root = output / "html"
    xml_root = output / "xml"
    site_index = root / "docs" / "site" / "index.md"
    if not site_index.is_file():
        print(
            "error: docs/site/index.md is missing; the documentation landing "
            "page is required before building the site.",
            file=sys.stderr,
        )
        return 2

    template_path = root / "docs" / "Doxyfile.in"
    if not template_path.is_file():
        print(f"error: missing Doxygen template: {template_path}", file=sys.stderr)
        return 2

    try:
        output.mkdir(parents=True, exist_ok=True)
        _refresh_owned_output(output, root)
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    config_dir = output / ".doxygen"
    config_dir.mkdir(parents=True, exist_ok=True)
    css = root / "docs" / "site" / "assets" / "lems-docs.css"
    js = root / "docs" / "site" / "assets" / "lems-docs.js"
    if not css.is_file() or not js.is_file():
        missing = [str(path.relative_to(root)) for path in (css, js) if not path.is_file()]
        print("warning: theme asset(s) missing; Doxygen will build without: " + ", ".join(missing), file=sys.stderr)

    guide_paths = sorted(root.glob("docs/*.md"))
    inputs = [root / "include" / "lems" / "data", root / "compat" / "vo", *guide_paths, root / "docs" / "site"]
    values = {
        "OUTPUT_DIRECTORY": _quote_config_path(output),
        "PROJECT_ROOT": _quote_config_path(root),
        "INPUT": _config_list(inputs),
        "MAINPAGE": _quote_config_path(site_index),
        "WARN_LOGFILE": _quote_config_path(output / "doxygen-warnings.log"),
        "HTML_EXTRA_STYLESHEET": _quote_config_path(css) if css.is_file() else "",
        "HTML_EXTRA_FILES": _quote_config_path(js) if js.is_file() else "",
    }
    template = template_path.read_text(encoding="utf-8")
    config_text = _replace_template(template, values)
    if args.no_xml:
        config_text = config_text.replace("GENERATE_XML           = YES", "GENERATE_XML           = NO")
    config_path = config_dir / "Doxyfile"
    config_path.write_text(config_text, encoding="utf-8")

    command = [resolved_doxygen, str(config_path)]
    print(f"Building Doxygen site into {html_root}")
    try:
        completed = subprocess.run(command, cwd=root, check=False)
    except OSError as exc:
        print(f"error: could not execute Doxygen at {resolved_doxygen}: {exc}", file=sys.stderr)
        return 2
    if completed.returncode != 0:
        print(f"error: Doxygen exited with status {completed.returncode}", file=sys.stderr)
        return completed.returncode or 1

    if not (html_root / "index.html").is_file():
        print(f"error: Doxygen completed without generating {html_root / 'index.html'}", file=sys.stderr)
        return 1
    changed = _postprocess_html(html_root, js.is_file(), root)
    marker = output / ".lems-docs-generated"
    marker.write_text(
        "Generated by tools/build_docs.py\n"
        f"source={root}\n"
        f"doxygen={resolved_doxygen}\n",
        encoding="utf-8",
    )
    if not args.keep_config:
        try:
            config_path.unlink()
            config_dir.rmdir()
        except OSError:
            pass
    warning_count = _warning_count(output / "doxygen-warnings.log")
    print(f"Built {html_root / 'index.html'} ({changed} pages received local theme script; Doxygen warnings: {warning_count})")
    if not args.no_xml:
        print(f"XML index: {xml_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
