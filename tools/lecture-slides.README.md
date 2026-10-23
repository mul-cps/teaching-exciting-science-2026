# Workshop slide build

The complete deck has 24 slides: eight introductory slides total 20 minutes,
followed by two editable slide cards for each of eight practical missions.
Both cards share the same mission duration; they add no session time. Each
card uses 22-point labels and 24-point body text. Complete practical details
remain in `docs/activity-cards.pdf` and the notebook. Text, notes, photos and timings come from
`curriculum/workshop.json`; layouts are native editable PowerPoint shapes/text
in `tools/build_lecture_slides.mjs`. The sequence diagram is also an editable
SVG at `docs/assets/fixed-fixture-sequence.svg`.

Install the renderer outside the repository:

```sh
npm install --prefix /tmp/workshop-slides-deps pptxgenjs@4.0.1
python tools/build_materials.py
python tools/build_notebooks.py --check
```

`PPTXGENJS_MODULE` can override the external module path; `SOFFICE` selects
LibreOffice. `LECTURE_EVIDENCE_DIR` selects slide PNGs and the build receipt
(default `/tmp/workshop-lecture-preview`). No classroom internet is needed.
Actual committed station photos are identified in the canonical source. The
local Blockly screenshot is labelled simulation; its invented demo recording
is not a physically taught student skill.
