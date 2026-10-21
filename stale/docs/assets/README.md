# Image provenance and usage

Historical record from 12 October 2026, archived on 21 October. Statements about “current” materials below describe that earlier version. See the [archive guide](../../README.md) for current replacements.

The repository includes **original teaching diagrams** (SVG/PNG) used in current materials and **archived AI-generated concept illustrations** (JPG source crops and source PNGs). No third-party photos are redistributed.

## Original teaching diagrams

`factory-overview`, `cps-loop`, `three-stations`, `safety-layout`, `vision-demo`, `sorting-rules`, `safe-sort-cycle`, `robot-demo`, `belt-demo`, `vision-roi`, `five-results`, and `fallback` are created with `python stale/tools/make_assets.py` from the repository's own SVG drawing code. Each illustration is a **schematic**, not a CAD layout or safety certification.

## Archived generated illustrative scenes

| Files | Origin | Limits |
|---|---|---|
| `sources/generated-overview-01.png`, `sources/generated-overview-02.png` | Images generated for this workshop with OpenAI image generation, 2026-10-12 | Composite *synthetic* posters with pseudo labels and code. **Do not interpret text or depicted connections as validated instructions.** |
| `archive/scene-*.jpg` | Cropped from the two images above using `python stale/tools/make_photo_assets.py` | For teaching illustration; not images of our real lab setup. Hardware may differ. |

Regenerate cropped photos using Pillow: `pip install Pillow && python stale/tools/make_photo_assets.py`.
Regenerate original schematic figures: `pip install cairosvg && python stale/tools/make_assets.py`.

**Public usage note:** Label AI-made scenes with “AI-generated concept illustration” where used in handouts, slides, websites, or posts. Do not present them as photographs of actual devices or as evidence that autonomous sorting has been physically validated. Generated visual representations of the university mark are not official university brand assets; don't use the source posters as official marketing artwork.

## External official image references (links only, not redistributed)

- [SO-ARM101 kit and assembled example — WowRobo](https://shop.wowrobo.com/products/so-arm101-diy-kit-assembled-version-1)
- [Factory Conveyor EV3 lesson — LEGO Education](https://education.lego.com/en-us/lessons/ev3-cim/make-a-factory-conveyor)
- [HERO10 Black manual — GoPro](https://gopro.com/content/dam/help/hero10-black/manuals/HERO10Black_UM_ENG_REVB.pdf)

Product photos shown by their original publishers are **not** automatically licensed for redistribution. Request permission or verify the applicable image license before adding external photographs to the repository.

### Current classroom visual policy

Use **only** the consistent vector teaching diagrams in PowerPoint, DOCX, and the main README. Generated image composites and cropped scene photos in this directory are kept solely as historical references, not teaching assets or accurate hardware evidence.
