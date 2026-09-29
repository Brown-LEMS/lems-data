#!/usr/bin/env python3
"""Validate local links and key API coverage in generated Doxygen HTML."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from html.parser import HTMLParser
import os
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_DOXYGEN_VERSION = "1.18.0"

REQUIRED_MEMBERS = {
    "Edge": ("location", "orientation", "frame_source", "index"),
    "Frame": ("timestamp_ns", "image_path", "edges", "ground_truth", "metadata", "synchronize_legacy_fields"),
    "CameraCalibration": ("resolution", "intrinsics", "K", "camera_matrix_cv", "distortion_cv"),
    "DatasetIterator": ("next", "reset", "has_next", "hasNext", "size"),
    "Utility": ("options", "get_Skew_Symmetric_Matrix", "get_edge_patches", "two_view_linear_triangulation"),
}


class LinkParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.links: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag.lower() != "a":
            return
        for name, value in attrs:
            if name.lower() == "href" and value:
                self.links.append(value)


class AssetParser(HTMLParser):
    """Collect local stylesheet and script references from a generated page.

    Doxygen's navigation is assembled from a small set of CSS and JavaScript
    files.  Checking those references separately from ordinary ``<a>`` links
    catches a site that is technically full of HTML pages but loses its
    navigation or styling when it is published below a project URL such as
    ``/lems-data/``.
    """

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.assets: list[tuple[str, str]] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        values = {name.lower(): value for name, value in attrs}
        tag = tag.lower()
        if tag == "link":
            rel = {part.lower() for part in (values.get("rel") or "").split()}
            href = values.get("href")
            if href and "stylesheet" in rel:
                self.assets.append(("stylesheet", href))
        elif tag == "script":
            src = values.get("src")
            if src:
                self.assets.append(("script", src))


class GeneratorParser(HTMLParser):
    """Read the Doxygen generator marker from the document head."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.generator: str | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag.lower() != "meta":
            return
        values = {name.lower(): value for name, value in attrs}
        if values.get("name", "").lower() == "generator":
            self.generator = values.get("content")


@dataclass
class _HomepageContainer:
    """A landing-page container collected while parsing generated HTML."""

    classes: set[str]
    parent: "_HomepageContainer | None" = None
    text: list[str] = field(default_factory=list)

    @property
    def nonempty(self) -> bool:
        return bool("".join(self.text).strip())


class HomepageParser(HTMLParser):
    """Collect the structural containers required by the landing page."""

    TRACKED = {"lems-hero", "lems-card-grid", "lems-card"}

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.containers: list[_HomepageContainer] = []
        self._open: list[tuple[str, _HomepageContainer | None]] = []

    @staticmethod
    def _classes(attrs: list[tuple[str, str | None]]) -> set[str]:
        for name, value in attrs:
            if name.lower() == "class" and value:
                return set(value.split())
        return set()

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        classes = self._classes(attrs)
        tracked = classes & self.TRACKED
        container: _HomepageContainer | None = None
        if tag.lower() == "div" and tracked:
            parent = next((node for _, node in reversed(self._open) if node is not None), None)
            container = _HomepageContainer(classes=classes, parent=parent)
            self.containers.append(container)
        self._open.append((tag.lower(), container))

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self.handle_starttag(tag, attrs)
        self.handle_endtag(tag)

    def handle_endtag(self, tag: str) -> None:
        tag = tag.lower()
        for index in range(len(self._open) - 1, -1, -1):
            if self._open[index][0] == tag:
                del self._open[index:]
                return

    def handle_data(self, data: str) -> None:
        for _, container in self._open:
            if container is not None:
                container.text.append(data)


def default_site(root: Path = ROOT) -> Path:
    for name in ("LEMS_DOCS_SITE", "LEMS_DOCS_HTML"):
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()
    return root.parent / "lems-data-docs-site" / "html"


def _symbol_files(html_root: Path, symbol: str) -> list[Path]:
    # Doxygen filenames are implementation details, so find pages by their
    # generated title rather than hard-coding struct/class filename encodings.
    matches: list[Path] = []
    title = re.compile(
        rf"<title>[^<]*\b{re.escape(symbol)}\b (?:Class|Struct) Reference</title>",
        re.I,
    )
    for page in html_root.rglob("*.html"):
        try:
            text = page.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if title.search(text):
            matches.append(page)
    return matches


def _missing_links(html_root: Path) -> list[tuple[Path, str, Path]]:
    missing: list[tuple[Path, str, Path]] = []
    for page in sorted(html_root.rglob("*.html")):
        try:
            parser = LinkParser()
            parser.feed(page.read_text(encoding="utf-8", errors="replace"))
        except OSError:
            continue
        for href in parser.links:
            parsed = urlsplit(href)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            target = (page.parent / unquote(parsed.path)).resolve()
            try:
                target.relative_to(html_root.resolve())
            except ValueError:
                missing.append((page, href, target))
                continue
            if not target.is_file():
                missing.append((page, href, target))
    return missing


def _missing_assets(html_root: Path) -> list[tuple[Path, str, str, Path | None]]:
    """Find missing or project-subpath-incompatible page assets.

    A reference beginning with ``/`` is rooted at the host domain and skips a
    GitHub Pages project prefix (for example, ``/lems-data/``).  Relative
    references are resolved from the page that contains them, including pages
    in Doxygen's ``search/`` and other subdirectories.  External, data, and
    fragment-only URLs are intentionally ignored.
    """

    missing: list[tuple[Path, str, str, Path | None]] = []
    resolved_root = html_root.resolve()
    for page in sorted(html_root.rglob("*.html")):
        try:
            parser = AssetParser()
            parser.feed(page.read_text(encoding="utf-8", errors="replace"))
        except OSError:
            continue
        for kind, reference in parser.assets:
            parsed = urlsplit(reference)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            path = unquote(parsed.path)
            if path.startswith("/"):
                missing.append((page, reference, f"root-relative {kind} URL", None))
                continue
            target = (page.parent / path).resolve()
            try:
                target.relative_to(resolved_root)
            except ValueError:
                missing.append((page, reference, f"{kind} escapes generated site", target))
                continue
            if not target.is_file():
                missing.append((page, reference, f"missing {kind}", target))
    return missing


def _doxygen_version(index_text: str) -> str | None:
    parser = GeneratorParser()
    parser.feed(index_text)
    if not parser.generator:
        return None
    match = re.match(r"Doxygen\s+(\S+)", parser.generator, re.I)
    return match.group(1) if match else None


def _check_homepage(index: Path) -> list[str]:
    """Enforce the small, intentionally stable landing-page structure."""

    parser = HomepageParser()
    try:
        parser.feed(index.read_text(encoding="utf-8", errors="replace"))
    except OSError as exc:
        return [f"could not read landing page: {exc}"]

    failures: list[str] = []
    heroes = [node for node in parser.containers if "lems-hero" in node.classes]
    grids = [node for node in parser.containers if "lems-card-grid" in node.classes]
    cards = [node for node in parser.containers if "lems-card" in node.classes]

    if len(heroes) != 1:
        failures.append(f"landing page must contain exactly one hero container (found {len(heroes)})")
    elif not heroes[0].nonempty:
        failures.append("landing page hero container is empty")

    if len(grids) != 1:
        failures.append(f"landing page must contain exactly one card grid (found {len(grids)})")
    else:
        grid = grids[0]
        outside = [card for card in cards if card.parent is not grid]
        if outside:
            failures.append("landing page card container is outside the single card grid")

    if len(cards) != 6:
        failures.append(f"landing page must contain exactly six cards (found {len(cards)})")
    empty = [str(index + 1) for index, card in enumerate(cards) if not card.nonempty]
    if empty:
        failures.append("landing page contains empty card(s): " + ", ".join(empty))
    return failures


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check generated lems-data Doxygen HTML")
    parser.add_argument("--site", type=Path, default=default_site(), help="generated HTML directory")
    parser.add_argument("--max-errors", type=int, default=20, help="maximum broken links to print")
    parser.add_argument(
        "--expected-doxygen-version",
        default=EXPECTED_DOXYGEN_VERSION,
        help=f"required Doxygen version in the generated HTML (default: {EXPECTED_DOXYGEN_VERSION})",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    html_root = args.site.expanduser().resolve()
    if not html_root.is_dir():
        print(f"error: generated HTML directory does not exist: {html_root}", file=sys.stderr)
        return 2
    index = html_root / "index.html"
    if not index.is_file():
        print(f"error: generated Doxygen index is missing: {index}", file=sys.stderr)
        return 1

    pages = sorted(html_root.rglob("*.html"))
    if len(pages) < 5:
        print(f"error: expected a real Doxygen site, found only {len(pages)} HTML page(s)", file=sys.stderr)
        return 1

    failures: list[str] = []
    index_text = index.read_text(encoding="utf-8", errors="replace")
    doxygen_version = _doxygen_version(index_text)
    if doxygen_version is None:
        failures.append("generated index does not identify its Doxygen version")
    elif doxygen_version != args.expected_doxygen_version:
        failures.append(
            f"generated site uses Doxygen {doxygen_version}; "
            f"expected Doxygen {args.expected_doxygen_version}"
        )
    if re.search(r"<h[1-6][^>]*>\s*@ref\b", index_text):
        failures.append("landing page contains an unresolved @ref command inside HTML")
    failures.extend(_check_homepage(index))
    for symbol, members in REQUIRED_MEMBERS.items():
        files = _symbol_files(html_root, symbol)
        if not files:
            failures.append(f"missing API symbol page: {symbol}")
            continue
        # Require the members on the symbol page itself; this prevents a
        # coincidental mention in a guide from satisfying coverage.
        combined = "\n".join(path.read_text(encoding="utf-8", errors="replace") for path in files)
        missing_members = [member for member in members if not re.search(rf"\b{re.escape(member)}\b", combined)]
        if missing_members:
            failures.append(f"{symbol} missing members: {', '.join(missing_members)}")

    search_assets = {
        path.name: path
        for path in html_root.rglob("*.js")
        if path.name in {"searchdata.js", "search.js"}
    }
    missing_search_assets = [name for name in ("searchdata.js", "search.js") if name not in search_assets]
    if missing_search_assets:
        failures.append("native Doxygen search index is missing: " + ", ".join(missing_search_assets))

    missing_assets = _missing_assets(html_root)
    if missing_assets:
        failures.append(f"{len(missing_assets)} broken local stylesheet/script asset reference(s)")
        for page, reference, reason, target in missing_assets[: max(0, args.max_errors)]:
            destination = f" ({target})" if target is not None else ""
            print(
                f"asset issue: {page.relative_to(html_root)} -> {reference} ({reason}){destination}",
                file=sys.stderr,
            )

    broken = _missing_links(html_root)
    if broken:
        failures.append(f"{len(broken)} broken local HTML file link(s)")
        for page, href, target in broken[: max(0, args.max_errors)]:
            print(f"broken link: {page.relative_to(html_root)} -> {href} ({target})", file=sys.stderr)

    if failures:
        print("documentation checks failed:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1

    print(
        f"documentation checks passed: {len(pages)} HTML pages, native search index, "
        "stylesheet/script assets, five API symbol/member surfaces, and no broken "
        "local HTML file links"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
