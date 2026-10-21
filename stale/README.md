# Historical workshop material

Archived on 21 October 2026 while preparing the 22 October workshop. Start with the [current workshop](../README.md) or the [documentation guide](../docs/README.md).

These files preserve earlier teaching designs and implementation history. They are not the class entry point or current station startup instructions.

| Location | Original location and purpose | Current replacement |
| --- | --- | --- |
| `docs/assets/` | `docs/assets/`: 12 schematic SVG/PNG pairs, original AI concept posters, cropped concepts and their provenance | [Lecture imagery and attribution](../docs/assets/lecture/README.md), notebook photos and current handouts |
| `docs/visual-gallery.md`, `docs/revision-report.md` | Same paths under the original `docs/`: 12 October schematic gallery and 20-slide revision report | [Current lecture notes](../docs/lecture-notes.md) and [workshop revision log](../docs/workshop-revision-log.md) |
| `docs/superpowers/` | `docs/superpowers/`: dated implementation plans, specs and progress records | [Station documentation](../docs/README.md); historical contracts remain linked where relevant |
| `tools/make_assets.py`, `tools/make_photo_assets.py` | Original `tools/` paths: generate old schematics and crop the old AI posters | [Current material builders](../tools/lecture-slides.README.md) and `tools/build_notebooks.py` |

The original relative directory structure is retained beneath `stale/`. For example, an old reference to `docs/superpowers/plans/…` is now `stale/docs/superpowers/plans/…`. Git history retains the previous paths. Evidence records that hash an original path describe the original snapshot; the move does not change that historical claim.

The two archived generators write beneath `stale/docs/assets/` when run directly. Current classroom builders do not import or execute them. The static developer inventory labels archived source separately.
