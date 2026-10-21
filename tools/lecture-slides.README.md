# Whole-class slide generator

`build_lecture_slides.mjs` creates the 18-slide `docs/workshop-slides.pptx`, its PDF, and `docs/presentation-preview.png`. Slides 1–10 form the 25-minute introduction. Slides 11–18 are timed activity checkpoints for the rest of the 240-minute workshop. All 18 teaching scripts are loaded from `docs/lecture-notes.md` into speaker notes.

Text, code, labels, and the requested teaching diagrams remain editable. The photographs, actual screenshots, and one explicitly labeled concept illustration are committed in `docs/assets/lecture/`. No private evidence directory or image-generation service is needed to rebuild.

Requirements: Node.js 22+, PptxGenJS 4.0.1, LibreOffice, Poppler's `pdftoppm`, ImageMagick's `magick`, Liberation Sans, and Noto Sans Mono. Install the JavaScript dependency outside the checkout:

```bash
npm install --prefix /tmp/workshop-slides-deps pptxgenjs@4.0.1
PPTXGENJS_MODULE=/tmp/workshop-slides-deps/node_modules/pptxgenjs/dist/pptxgen.cjs.js \
  node tools/build_lecture_slides.mjs
```

Set `SOFFICE` to an alternative LibreOffice executable. Set `LECTURE_EVIDENCE_DIR` to choose the private location of 18 full-slide PNGs and the build receipt. Its default is `/tmp/workshop-lecture-preview`. If PptxGenJS is installed in the checkout's Node resolution path, its environment variable is unnecessary.

The preferred artifact-tool runtime was unavailable in the authoring session. The supervising agent selected this isolated JavaScript fallback. The generator removes PptxGenJS 4.0.1's dangling shared-master content-type declarations and preserves the native editable slide objects.
