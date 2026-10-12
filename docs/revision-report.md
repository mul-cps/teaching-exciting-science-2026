# Revision log · five-pass design and reliability audit

Revision date: 2026-10-12. Scope: classroom PowerPoint, DOCX handouts, included visual assets, and teaching code consistency.

## Pass 1 — Story and technical correctness

- Audited the sorter against `workshop/factory.py` and all four exercises.
- Corrected a reverse red color route in the original system schematic to **red → left**, **blue → right**, **unknown → reject**.
- Corrected an old demo diagram that previously sent unknown colors to the wrong destination.
- Aligned station letters **A = robot, B = vision, C = EV3 conveyor** across slides and handouts.
- Real hardware remains explicitly NOT VALIDATED, with simulation the required reliable fallback.

## Pass 2 — Visual consistency and provenance

- Replaced mismatched AI-generated photo crops with a consistent original vector illustration family.
- Added five new SVG + PNG illustrations: `robot-demo`, `belt-demo`, `vision-roi`, `five-results`, `fallback`.
- Retained previous AI composites/crops **only in archive** for reference and provenance; no longer shown in final PPT/DOCX.
- One navy/teal/grey palette, consistent red/blue color coding, repeated camera/arm/belt silhouettes.

## Pass 3 — PowerPoint reconstruction

- Rebuilt as 20 editable 16:9 slides using `python-pptx`.
- One header/footer/title grid, consistent typography, diagram placement, and native editable code/cards.
- Separate slides for safety, three rotation stations, Python exercises, safe sequence, five-object test, reflection and simulation fallback.
- Code snippets are valid extracts/examples of existing classes and methods rather than imaginary hardware commands.

## Pass 4 — Document synchronization

- Rebuilt instructor guide and student worksheet with matching graphics and typography.
- Instructor guide: 2 pages including rotation, preparation, safety cycle, and unverified hardware caveat.
- Student worksheet: 2 pages with exact scripts, input fields, and five-object exercise.
- Both rendered as pages for visual inspection; no detected page clipping or spilled tables.

## Pass 5 — Tests and packaging

- `python -m pytest -q`: **9 passed** (software unit tests only).
- `python -m workshop`: printed `{'left': 2, 'right': 2, 'reject': 1}` for the default five-object test.
- All four exercise scripts executed without errors on the current Python environment.
- PowerPoint rendered successfully: **20 slides**. `slides_test.py`: **Test passed. No overflow detected.**
- Both DOCX files rendered successfully: **2 pages each**.
- Removed obsolete snapshots, old slide PNG cache, `__pycache__`, and `.pytest_cache` from the distributable repository.
- Added Pillow and CairoSVG to `[materials]` extras for reproducible document and diagram regeneration.

## Important limitations

- Layout was checked using LibreOffice in this environment. Microsoft PowerPoint rendering can differ slightly (fonts and text wrapping).
- None of the real SO-ARM101, LEGO EV3, GoPro HERO10, or autonomous gripping paths has been bench validated in this environment. These remain TODO in `docs/acceptance.md`.
- SVG source diagrams are editable. Embedded images in PowerPoint are raster renditions; card text and Python code are native editable shapes.
- Earlier AI illustrations are **archived**, not corrected representations of specific real hardware.

## Rebuild commands

```bash
pip install -e '.[materials,test]'
python tools/make_assets.py
python tools/build_materials.py
pytest -q
```
