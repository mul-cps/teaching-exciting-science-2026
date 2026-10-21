"""Build shared notebooks, DOCX/PDF handouts and the maintained JavaScript slides.

Set SOFFICE to the prepared LibreOffice executable for PDF export.
Use --handouts-only when the slide deck is already built.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ART = DOCS / "assets"
NAVY = "17354B"
TEAL = "197F80"
GREY = "607383"
PALE = "E4EFF0"
WHITE = "FFFFFF"
BG = "F7FAFB"
GOLD = "BF8D42"
RED = "DC6658"


def make_docx(filename, source, *, worksheet=False):
    """Render the Markdown handout source without maintaining a second lesson plan."""
    import re
    from docx import Document
    from docx.shared import Inches, Pt, RGBColor
    from docx.oxml import OxmlElement
    from docx.oxml.ns import qn

    d = Document()
    sec = d.sections[0]
    sec.page_width, sec.page_height = Inches(8.27), Inches(11.69)
    sec.top_margin = sec.bottom_margin = Inches(.45 if worksheet else .65)
    sec.left_margin = sec.right_margin = Inches(.58 if worksheet else .72)
    for name, size in (("Normal", 10), ("Title", 20 if worksheet else 24),
                       ("Heading 1", 11 if worksheet else 15), ("Heading 2", 10 if worksheet else 12)):
        st = d.styles[name]
        st.font.name, st.font.size = "DejaVu Sans", Pt(size)
        st.font.color.rgb = RGBColor.from_string(NAVY)
        st.paragraph_format.space_after = Pt(3 if worksheet else 6)
    for style in d.styles:
        for border in style.element.findall(".//" + qn("w:pBdr")):
            border.getparent().remove(border)
    d.styles["Normal"].paragraph_format.line_spacing = 1.0 if worksheet else 1.08
    d.styles["Heading 1"].paragraph_format.space_before = Pt(7)
    d.styles["Title"].paragraph_format.space_after = Pt(6)
    d.styles["Title"].font.color.rgb = RGBColor(0, 0, 0)

    def clean(text):
        text = re.sub(r"\[([^]]+)\]\([^)]+\)", r"\1", text)
        return text.replace("**", "").replace("`", "")

    def runs(p, text, code=False):
        if code:
            run = p.add_run(text)
            run.font.name, run.font.size = "DejaVu Sans Mono", Pt(8.5)
            return
        for i, part in enumerate(re.split(r"(`[^`]+`)", text)):
            run = p.add_run(clean(part))
            if part.startswith("`"):
                run.font.name, run.font.size = "DejaVu Sans Mono", Pt(9 if worksheet else 9)

    lines = (DOCS / source).read_text().splitlines()
    table = None
    code = False
    for line in lines:
        if line.startswith("```"):
            code = not code
            continue
        if not line.strip():
            table = None
            continue
        if code:
            runs(d.add_paragraph(), line, code=True)
            continue
        if line.startswith("!["):
            match = re.match(r"!\[([^]]*)\]\(([^)]+)\)", line)
            if match:
                d.add_picture(str((DOCS / match.group(2)).resolve()), width=Inches(6.6))
                continue
        if line.startswith("|"):
            values = [clean(v.strip()) for v in line.strip("|").split("|")]
            if all(re.fullmatch(r":?-+:?", v) for v in values):
                continue
            if table is None:
                table = d.add_table(rows=0, cols=len(values))
                table.style = "Light Shading Accent 1"
            cells = table.add_row().cells
            for cell, value in zip(cells, values):
                runs(cell.paragraphs[0], value)
                for run in cell.paragraphs[0].runs:
                    run.font.size = Pt(8)
            if len(table.rows) == 1:
                table.rows[0]._tr.get_or_add_trPr().append(OxmlElement("w:tblHeader"))
            continue
        if line.startswith("# "):
            d.add_paragraph(clean(line[2:]), "Title")
        elif line.startswith("### "):
            d.add_paragraph(clean(line[4:]), "Heading 2")
        elif line.startswith("## "):
            if filename == "student-handbook.docx" and line.startswith(("## A ", "## B ", "## C ", "## D ", "## E ")):
                d.add_page_break()
            d.add_paragraph(clean(line[3:]), "Heading 1")
        elif line.startswith("- [ ] "):
            runs(d.add_paragraph(), "[ ] " + line[6:])
        else:
            p = d.add_paragraph()
            runs(p, line, code=code)
    d.core_properties.title = (DOCS / source).read_text().splitlines()[0].lstrip("# ")
    d.core_properties.subject = "22 October 2026 workshop"
    d.save(DOCS / filename)


def make_slides():
    """Delegate to the maintained JavaScript slide generator."""
    import subprocess
    subprocess.run(["node", str(ROOT / "tools/build_lecture_slides.mjs")], cwd=ROOT, check=True)
    return DOCS / "workshop-slides.pptx"


def export_handout_pdfs():
    """Use an explicit SOFFICE when the classroom build uses portable LibreOffice."""
    import os
    import shutil
    import subprocess
    import tempfile
    soffice = os.environ.get("SOFFICE") or shutil.which("soffice")
    if not soffice:
        raise RuntimeError("Set SOFFICE to the prepared LibreOffice executable to export handout PDFs")
    for name in ("instructor-guide", "student-worksheet", "student-handbook", "lesson-plan"):
        with tempfile.TemporaryDirectory(prefix="workshop-handout-") as profile:
            subprocess.run([soffice, "-env:UserInstallation=" + Path(profile).as_uri(), "--headless", "--convert-to", "pdf", "--outdir", str(DOCS), str(DOCS/(name + ".docx"))], check=True, capture_output=True)
        if not (DOCS/(name + ".pdf")).is_file():
            raise RuntimeError("PDF export missing: " + name)


def main():
    import argparse
    from build_notebooks import main as notebooks
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handouts-only", action="store_true")
    args = parser.parse_args()
    notebooks()
    make_docx("instructor-guide.docx", "instructor.md")
    make_docx("student-worksheet.docx", "student-worksheet.md", worksheet=True)
    make_docx("student-handbook.docx", "student-exercises.md")
    make_docx("lesson-plan.docx", "lesson-plan.md")
    export_handout_pdfs()
    if not args.handouts_only:
        make_slides()
    print("Built handouts and notebooks in", DOCS, "and", ROOT/"notebooks")


if __name__ == "__main__":
    main()
