#!/usr/bin/env python3
"""Build a local, static reference for every tracked source/configuration file.

Griffe visits Python syntax without importing project modules. Doxygen parses
C++ without building/linking ROS or Qt. MkDocs renders the generated Markdown.
Run ``python tools/build_code_docs.py --help`` for the local build commands.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "build/code-docs"
SKIP_PARTS = {".git", ".venv", "venv", "node_modules", "build", "install", "log", "graft", "__pycache__", ".cache"}
LANGUAGES = {
    ".py": "python", ".cpp": "cpp", ".hpp": "cpp", ".h": "cpp",
    ".sh": "bash", ".js": "javascript", ".mjs": "javascript", ".ts": "typescript",
    ".slang": "cpp", ".yaml": "yaml", ".yml": "yaml", ".json": "json",
    ".toml": "toml", ".cfg": "ini", ".xml": "xml", ".urdf": "xml",
    ".srdf": "xml", ".rviz": "yaml", ".repos": "yaml", ".ipynb": "notebook", ".srv": "text",
    ".msg": "text", ".action": "text", ".patch": "diff",
}


def tracked_files(root: Path = ROOT) -> list[Path]:
    """Return sorted Git-owned files, excluding generated and dependency trees."""
    result = subprocess.run(["git", "ls-files", "-z"], cwd=root, check=True, capture_output=True)
    return sorted({Path(p) for p in result.stdout.decode().split("\0") if p and not SKIP_PARTS.intersection(Path(p).parts)})


def language(path: Path, text: str) -> str | None:
    """Classify executable wrappers as well as conventional source suffixes."""
    if path.suffix in LANGUAGES:
        return LANGUAGES[path.suffix]
    if path.name.startswith("requirements") and path.suffix == ".txt":
        return "text"
    if path.name in {"CMakeLists.txt", "Containerfile", "Dockerfile"}:
        return "cmake" if path.name == "CMakeLists.txt" else "dockerfile"
    if text.startswith("#!"):
        return "python" if "python" in text.splitlines()[0] else "bash"
    return None


def page_name(path: Path) -> str:
    """Preserve the complete original path; suffixes prevent stem collisions."""
    return "source/" + "/".join("_dot_" + part[1:] if part.startswith(".") else part for part in path.parts) + ".md"


def fenced(text: str, lang: str = "text") -> str:
    """Use a fence longer than any source backtick run."""
    runs = re.findall(r"`+", text)
    fence = "`" * max(3, max(map(len, runs), default=0) + 1)
    return f"{fence}{lang}\n{text.rstrip()}\n{fence}\n"


def python_reference(path: Path, code: str) -> tuple[str, int, list[dict]]:
    """Extract real signatures and docstrings using Griffe's static visitor."""
    from griffe import visit

    module = visit(path.stem.replace(".", "_"), filepath=path, code=code)
    sections = ["## Python API\n", "Extracted by Griffe; imported aliases are omitted. Private helpers are included.\n"]
    count = 0
    if module.docstring:
        sections.append(fenced(module.docstring.value))

    def render(obj, prefix=""):
        nonlocal count
        for member in obj.members.values():
            if member.is_alias or member.is_module:
                continue
            name = prefix + member.name
            if not (member.is_class or member.is_function):
                continue
            count += 1
            if member.is_function:
                signature = ("async " if "async" in member.labels else "") + "def " + member.signature(name=name)
            else:
                signature = f"class {name}" + ("(" + ", ".join(map(str, member.bases)) + ")" if member.bases else "")
            sections.extend([f"### `{name}`\n", fenced(signature, "python"), f"[Source, line {member.lineno}](#source-line-{member.lineno})\n"])
            sections.append(fenced(member.docstring.value) if member.docstring else "No docstring is declared; consult the source below.\n")
            if member.is_class:
                render(member, name + ".")

    render(module)
    return "\n".join(sections), count, ros_calls(code)


def ros_calls(code: str) -> list[dict]:
    """List declared ROS endpoints/parameters with source expressions and lines.

    Expressions are intentionally not evaluated: a dynamic topic remains an
    expression, rather than being advertised as a resolved runtime graph.
    """
    calls = []
    recognized = {"create_publisher", "create_subscription", "create_service", "create_client", "ActionServer", "ActionClient", "declare_parameter", "declare_launch_argument", "DeclareLaunchArgument"}
    for node in ast.walk(ast.parse(code)):
        if not isinstance(node, ast.Call):
            continue
        name = node.func.attr if isinstance(node.func, ast.Attribute) else node.func.id if isinstance(node.func, ast.Name) else ""
        if name in recognized:
            calls.append({"kind": name, "line": node.lineno, "expression": ast.unparse(node)})
    return sorted(calls, key=lambda item: item["line"])


def write(path: str, content: str) -> None:
    target = OUTPUT / "docs" / path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content)


def generate() -> dict:
    """Generate Markdown and a machine-readable exhaustive coverage manifest."""
    shutil.rmtree(OUTPUT / "docs", ignore_errors=True)
    records, skipped, endpoints = [], [], []
    for path in tracked_files():
        raw = (ROOT / path).read_bytes()
        try:
            code = raw.decode("utf-8")
        except UnicodeDecodeError:
            skipped.append({"path": str(path), "reason": "binary asset"})
            continue
        lang = language(path, code)
        if lang is None:
            skipped.append({"path": str(path), "reason": "prose, license, data, or non-code artifact"})
            continue
        api, symbols, calls = "", 0, []
        source_code = code
        source_lang = lang
        if lang == "notebook":
            notebook = json.loads(code)
            cells = ["".join(cell.get("source", [])) for cell in notebook.get("cells", []) if cell.get("cell_type") == "code"]
            source_code = "\n\n".join(cells)
            source_lang = "python"
            try:
                api, symbols, calls = python_reference(path, source_code)
            except SyntaxError:
                api = "## Notebook cells\n\nIPython syntax prevents plain-Python API extraction; complete code cells follow.\n"
        if lang == "python":
            # No importlib, loader inspection, subprocess execution or alias resolution.
            api, symbols, calls = python_reference(path, code)
        for call in calls:
            endpoints.append({"path": str(path), **call})
        lines = "\n".join(f'<span id="source-line-{n}"></span>' for n in sorted({c["line"] for c in calls}))
        # Every API source link has an HTML anchor, independent of code highlighting.
        anchors = set(map(int, re.findall(r"#source-line-(\d+)", api)))
        lines += "\n" + "\n".join(f'<span id="source-line-{n}"></span>' for n in sorted(anchors))
        write(page_name(path), f"# `{path}`\n\n" + ("Archived historical source; current workshop builders do not use this file.\n\n" if path.parts[0] == "stale" else "") + f"[Full inventory]({'../' * (len(Path(page_name(path)).parts) - 1)}inventory.md)\n\nLanguage: {lang}. SHA-256: `{hashlib.sha256(raw).hexdigest()}`.\n\n{api}\n\n## Source\n\nThe complete file is shown with original line numbers.\n\n{lines}\n\n{fenced(source_code, source_lang)}")
        records.append({"status": "archived" if path.parts[0] == "stale" else "active", "path": str(path), "language": lang, "page": page_name(path), "symbols": symbols, "sha256": hashlib.sha256(raw).hexdigest()})
    counts = Counter(r["language"] for r in records)
    report = {"schema": 1, "discovery": "git ls-files; generated/dependency trees excluded", "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(), "files": records, "excluded": skipped, "counts": dict(sorted(counts.items())), "uncovered_code_files": []}
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "coverage.json").write_text(json.dumps(report, indent=2) + "\n")
    for path in sorted((ROOT / "docs/code-reference").glob("*.md")):
        write(path.name, path.read_text())
    inventory = ["# Full source inventory\n", "Every entry links to the complete highlighted source. Python entries include Griffe API signatures and declared docstrings. C++ is also parsed by Doxygen when available.\n"]
    for group in sorted({Path(r["path"]).parts[0] for r in records}):
        inventory += [f"\n## {group}\n" + ("\nHistorical source retained for provenance; not a current classroom entry point.\n" if group == "stale" else ""), "| File | Language | Python classes/functions |", "| --- | --- | ---: |"]
        inventory += [f'| [{r["path"]}]({r["page"]}) | {r["language"]} | {r["symbols"]} |' for r in records if Path(r["path"]).parts[0] == group]
    write("inventory.md", "\n".join(inventory))
    coverage = ["# Coverage report\n", f"Source commit: `{report['source_commit']}`. Inventory reads current tracked working-tree contents; file hashes record local edits.\n", f"**{len(records)} source/configuration files; {sum(r['symbols'] for r in records)} Python classes/functions; 0 uncovered classified code files.**\n", "| Language | Files |\n| --- | ---: |", *[f"| {key} | {value} |" for key, value in sorted(counts.items())], "\n## Explicit exclusions\n", "Generated/build/dependency trees are excluded by path. Every other Git file omitted from the API inventory is listed below so classification can be audited. The JSON manifest is `build/code-docs/coverage.json`. No docstring is fabricated where a source lacks one. Static endpoint extraction does not prove runtime reachability, hardware readiness, or physical acceptance.\n", "| File | Reason |\n| --- | --- |", *[f"| `{r['path']}` | {r['reason']} |" for r in skipped]]
    write("coverage.md", "\n".join(coverage))
    contracts = [(ROOT / "docs/code-reference/ros-contracts-intro.txt").read_text(), "\n## Declared endpoints and parameters\n", "Exact source expressions below include defaults and dynamic names. They are a static declaration index, not a live ROS graph.\n"]
    for row in endpoints:
        contracts += [f"### `{row['path']}:{row['line']}` — `{row['kind']}`\n", f"[Source]({page_name(Path(row['path']))}#source-line-{row['line']})\n", fenced(row["expression"], "python")]
    contracts += ["## Service, message and action schemas\n"]
    for record in records:
        if Path(record["path"]).suffix in {".srv", ".msg", ".action"}:
            contracts += [f"### `{record['path']}`\n", f"[Source]({record['page']})\n", fenced((ROOT / record["path"]).read_text())]
    write("ros-contracts.md", "\n".join(contracts))
    return report


def doxygen() -> bool:
    """Generate native APIs with the local Doxygen binary, never a C++ build."""
    binary = shutil.which("doxygen")
    if not binary:
        return False
    sources = [str(ROOT / p) for p in tracked_files() if p.suffix in {".cpp", ".hpp", ".h"}]
    config = '\n'.join([
        'PROJECT_NAME = "CPS Candy Factory native API"',
        f'OUTPUT_DIRECTORY = "{OUTPUT / "doxygen"}"',
        'INPUT = ' + ' '.join('"' + p + '"' for p in sources),
        'GENERATE_HTML = YES', 'GENERATE_LATEX = NO', 'HTML_OUTPUT = html',
        'EXTRACT_ALL = YES', 'EXTRACT_PRIVATE = YES', 'EXTRACT_STATIC = YES',
        'SOURCE_BROWSER = YES', 'INLINE_SOURCES = YES', 'HAVE_DOT = NO',
        'QUIET = YES', 'WARNINGS = NO', 'WARN_IF_UNDOCUMENTED = NO',
        'MACRO_EXPANSION = YES', 'PREDEFINED = Q_OBJECT=',
    ])
    subprocess.run([binary, "-"], input=config, text=True, check=True, cwd=ROOT)
    return True


def check_site(report: dict) -> None:
    """Check all generated pages and local HTML href/src targets and anchors."""
    from html.parser import HTMLParser
    from urllib.parse import unquote, urlsplit

    site = OUTPUT / "site"
    missing = []
    class Links(HTMLParser):
        def __init__(self):
            super().__init__(); self.links = []; self.ids = set()
        def handle_starttag(self, tag, attrs):
            data = dict(attrs)
            self.ids.update([data["id"]] if "id" in data else [])
            if "name" in data: self.ids.add(data["name"])
            self.links += [data[key] for key in ("href", "src") if key in data]
    pages = {}
    for path in site.rglob("*.html"):
        parser = Links(); parser.feed(path.read_text()); pages[path.resolve()] = parser
    for path, parser in pages.items():
        for link in parser.links:
            url = urlsplit(html.unescape(link))
            if url.scheme or url.netloc or link.startswith("/"): continue
            target = (path.parent / unquote(url.path)).resolve() if url.path else path
            if target.is_dir(): target /= "index.html"
            if not target.exists(): missing.append(f"{path.relative_to(site)} -> {link}")
            elif url.fragment and target in pages and unquote(url.fragment) not in pages[target].ids:
                missing.append(f"{path.relative_to(site)} -> missing anchor {link}")
    for row in report["files"]:
        if not (site / Path(row["page"]).with_suffix(".html")).exists():
            missing.append(row["path"])
    if missing:
        raise RuntimeError("Broken generated links:\n" + "\n".join(missing[:30]))
    print(f"Verified {len(report['files'])} source pages; {len(pages)} HTML pages; local links and anchors resolve.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generate-only", action="store_true", help="write Markdown and coverage without rendering")
    parser.add_argument("--no-doxygen", action="store_true", help="skip optional native API extraction; source pages remain")
    args = parser.parse_args()
    report = generate()
    if args.generate_only:
        print(json.dumps(report["counts"], sort_keys=True)); return
    native = not args.no_doxygen and doxygen()
    # The hyperlink is included only if its output actually exists.
    with (OUTPUT / "docs/index.md").open("a") as stream:
        stream.write('\n<a href="doxygen/index.html">Doxygen C++ API (including Qt/RViz panel)</a>\n' if native else '\nDoxygen is unavailable or disabled; native APIs have complete source pages in the inventory.\n')
    subprocess.run([sys.executable, "-m", "mkdocs", "build", "--strict", "-f", str(ROOT / "mkdocs.yml")], check=True, cwd=ROOT)
    if native:
        shutil.copytree(OUTPUT / "doxygen/html", OUTPUT / "site/doxygen", dirs_exist_ok=True)
    check_site(report)


if __name__ == "__main__":
    main()
