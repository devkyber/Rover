"""Check maintained Rover Markdown; run from any directory, with no dependencies.

Historical reviews, archives, the submitted proposal, generated analysis reports,
vendor material and machine-local notes are intentionally outside this check.
Supports this repository's inline links, ATX headings and explicit HTML anchors.
Sibling PCB links are checked when that checkout is present.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import re
import sys
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
ROADMAP = Path("docs/05_한계와_로드맵.md")
LINK = re.compile(r"!?\[[^\]\n]*\]\((<[^>]+>|[^)\n]+)\)")
EXPLICIT_ID = re.compile(r'<a\s+(?:id|name)=[\"\x27]([^\"\x27]+)[\"\x27]')
TASK_ID = re.compile(r"[a-z]{2,}-\d{2}\Z")  # F-01 etc. are failure-history entries.
TASK_TITLE = re.compile(r"^\*\*([A-Z]{2,}-\d{2})\s+[—-]", re.MULTILINE)
CHECKBOX = re.compile(r"^\s*[-*+]\s+\[[ xX]\]", re.MULTILINE)


def prose(source: str) -> str:
    """Blank fenced blocks while preserving source line numbers."""
    result, fence = [], None
    for line in source.splitlines():
        marker = re.match(r"\s*(`{3,}|~{3,})", line)
        if fence:
            if marker and marker[1][0] == fence[0] and len(marker[1]) >= len(fence):
                fence = None
            result.append("")
        elif marker:
            fence = marker[1]
            result.append("")
        else:
            result.append(line)
    return "\n".join(result)


def anchors(source: str) -> set[str]:
    text = prose(source)
    found = set(EXPLICIT_ID.findall(text))
    counts: Counter[str] = Counter()
    for heading in re.findall(r"^#{1,6}\s+(.+)$", text, re.MULTILINE):
        heading = re.sub(r"\s+#+\s*$", "", heading)
        heading = re.sub(r"<[^>]*>", "", heading).lower()
        slug = re.sub(r"[^\w\- ]", "", heading).replace(" ", "-")
        suffix = "" if counts[slug] == 0 else f"-{counts[slug]}"
        found.add(slug + suffix)
        counts[slug] += 1
    return found


def maintained_files(root: Path) -> list[Path]:
    files = set(root.glob("*.md"))
    files.update(root.glob("docs/**/*.md"))
    files.update(root.glob("data/**/*.md"))
    files.update(root.glob("firmware/**/README.md"))
    files.update(root / rel for rel in ("station/README.md", "logs/README.md", "data/README.md"))
    excluded = (root / "docs/archive", root / "docs/reviews")
    return sorted(p for p in files if p.is_file()
                  and not p.name.endswith(".local.md")
                  and p.name != "91_KSRC_기술제안서.md"
                  and not any(p.is_relative_to(folder) for folder in excluded))


def check(root: Path) -> tuple[list[str], int, int, int]:
    root = root.resolve()
    errors, skipped = [], set()
    files = maintained_files(root)
    texts = {p: p.read_text(encoding="utf-8-sig") for p in files}
    roadmap = root / ROADMAP
    if roadmap not in texts:
        errors.append(f"Missing canonical roadmap: {ROADMAP.as_posix()}")
    total_links = 0
    for path, source in texts.items():
        rel, text = path.relative_to(root).as_posix(), prose(source)
        if path != roadmap and re.search(r"(?:^|[_-])(todo|todos|status|roadmap|backlog)(?:$|[_-])", path.stem.lower()):
            errors.append(f"{rel}: parallel status file; update the canonical roadmap")
        if path != roadmap and CHECKBOX.search(text):
            errors.append(f"{rel}: task checklist outside the roadmap; link its task ID")
        ids = EXPLICIT_ID.findall(text)
        for name, count in Counter(ids).items():
            if count > 1:
                errors.append(f"{rel}: duplicate anchor {name}")
            if TASK_ID.fullmatch(name) and path != roadmap:
                errors.append(f"{rel}: task {name} defined outside the roadmap")
        titles = [name.lower() for name in TASK_TITLE.findall(text)]
        for name, count in Counter(titles).items():
            if path != roadmap or count != 1 or ids.count(name) != 1:
                errors.append(f"{rel}: task {name} needs one title/anchor in the roadmap")
        for match in LINK.finditer(text):
            target = match[1].strip()
            target = target[1:-1] if target.startswith("<") else target.split(' "', 1)[0]
            if re.match(r"[a-zA-Z][a-zA-Z0-9+.-]*:", target) or target.startswith("//"):
                continue
            name, _, fragment = unquote(target).partition("#")
            destination = (path.parent / name).resolve() if name else path
            where = f"{rel}:{text[:match.start()].count(chr(10)) + 1}"
            total_links += 1
            if not destination.is_relative_to(root):
                pcb = root.parent / "PCB"
                if destination.is_relative_to(pcb) and not pcb.exists():
                    skipped.add(target)
                    continue
                if not destination.is_relative_to(pcb):
                    errors.append(f"{where}: local link escapes Rover/PCB: {target}")
                    continue
            if not destination.exists():
                errors.append(f"{where}: missing local target {target}")
            elif fragment and destination.suffix == ".md":
                destination_text = texts.get(destination)
                if destination_text is None:
                    destination_text = destination.read_text(encoding="utf-8-sig")
                if fragment not in anchors(destination_text):
                    errors.append(f"{where}: missing Markdown anchor {target}")
    return errors, len(files), total_links, len(skipped)


def main() -> int:
    # Korean filenames must also render when Windows launches Python with pipes.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    errors, count, links, skipped = check(ROOT)
    for error in errors:
        print(error)
    if skipped:
        print(f"NOTE: {skipped} sibling PCB links not checked (checkout absent).")
    print(f"{'FAIL' if errors else 'PASS'}: {count} maintained documents, {links} local links, {len(errors)} errors.")
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
