# Developer code reference

This reference covers the offline workshop, exercises, tools, ROS packages,
Python tests, native backends, launch files, services, shell commands and
configuration. Start with the [architecture](architecture.md), then use the
[full source inventory](inventory.md) or search to find a module or symbol.
[ROS contracts](ros-contracts.md) lists declarations and exact service schemas.
The [coverage report](coverage.md) lists counts and every excluded Git file.

## Build locally

From the repository root:

```bash
python -m venv /tmp/cps-code-docs-venv
/tmp/cps-code-docs-venv/bin/pip install -r requirements-docs.txt
/tmp/cps-code-docs-venv/bin/python tools/build_code_docs.py
/tmp/cps-code-docs-venv/bin/python -m http.server 8000 --directory build/code-docs/site
```

Open `http://localhost:8000`. Stop the file server with Ctrl+C.
Doxygen is optional (`doxygen` on PATH); when installed it generates native
class/function and source references including the Qt/RViz operator panel.
The Python build uses **MkDocs, Griffe and Pygments** through pymdownx.
Griffe's static visitor reads syntax and docstrings directly: it never imports
project modules, resolves imported aliases, or loads ROS, camera, GPU or robot
SDKs. Extensionless Python ROS wrappers and notebook code cells are included.
Notebook cells receive Griffe APIs when they parse as ordinary Python;
IPython-specific syntax retains a complete code-cell source fallback. C++ is parsed by Doxygen
without compiling or linking it. Other languages and configuration receive
complete highlighted source pages. JavaScript/TypeScript receive source pages;
this build does not claim semantic JSDoc/TSDoc extraction.

`--generate-only` writes Markdown and `build/code-docs/coverage.json`.
`--no-doxygen` explicitly selects the source-only native fallback.
The normal command performs a strict MkDocs build and checks generated local
HTML links, source anchors and page coverage. It launches no station processes.

## Scope and provenance

Discovery uses `git ls-files`; commit or stage new sources before building to
include them. Ignored virtual environments, vendor/cache, generated build,
install and log trees are excluded. `coverage.json` records the source commit
and SHA-256 of every included file, including working-tree changes. Build
outputs stay in `build/code-docs`; they are not committed or published.

An API declaration describes software behavior. It does not establish physical
calibration, tracking, grasp success, motion safety or station acceptance.
Consult the repository's `docs/acceptance.md` and subsystem commissioning
guides before physical use. This reference does not run code examples.

Documentation library references:
[Griffe static visitor](https://mkdocstrings.github.io/griffe/reference/api/agents/visitor/),
[MkDocs](https://www.mkdocs.org/),
[Doxygen](https://www.doxygen.nl/manual/starting.html).
