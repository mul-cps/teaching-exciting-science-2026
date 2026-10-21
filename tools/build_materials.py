"""Build aligned instructor DOCX, student DOCX, and PPTX from editable assets.

Use `python tools/make_assets.py` to regenerate the SVG/PNG diagrams first.
Requires only the optional `materials` Python dependencies.
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
        elif line.startswith("## "):
            if not worksheet and line.startswith(("## 2.", "## 4.")):
                d.add_page_break()
            d.add_paragraph(clean(line[3:]), "Heading 1")
        elif line.startswith("- [ ] "):
            runs(d.add_paragraph(), "[ ] " + line[6:])
        else:
            p = d.add_paragraph()
            runs(p, line, code=code)
    d.core_properties.title = "Autonomous Robot Sorting"
    d.core_properties.subject = "22 October 2026 workshop"
    d.save(DOCS / filename)


def make_slides():
    from pptx import Presentation
    from pptx.util import Inches, Pt
    from pptx.dml.color import RGBColor
    from pptx.enum.shapes import MSO_SHAPE
    from pptx.enum.text import MSO_ANCHOR
    from PIL import Image

    prs = Presentation()
    W, H = 13.333, 7.5
    prs.slide_width, prs.slide_height = Inches(W), Inches(H)
    C = lambda x: RGBColor.from_string(x)

    def txt(s, x, y, w, h, text, size=20, color=NAVY, bold=False, mono=False):
        sh = s.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
        tf = sh.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_right = Inches(.025)
        tf.margin_top = tf.margin_bottom = Inches(.015)
        for i, line in enumerate(text.split("\n")):
            p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
            p.text = line
            p.font.name = "DejaVu Sans Mono" if mono else "DejaVu Sans"
            p.font.size = Pt(size)
            p.font.bold = bold
            p.font.color.rgb = C(color)
            p.space_after = Pt(5 if mono else 0)
        return sh

    def rect(s,x,y,w,h,fill=PALE,stroke=None,r=0.14):
        sh=s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE if r else MSO_SHAPE.RECTANGLE, Inches(x),Inches(y),Inches(w),Inches(h))
        sh.fill.solid();sh.fill.fore_color.rgb=C(fill)
        if stroke: sh.line.color.rgb=C(stroke)
        else: sh.line.fill.background()
        return sh

    def slide(title, chapter="OUR FACTORY"):
        s=prs.slides.add_slide(prs.slide_layouts[6])
        s.background.fill.solid();s.background.fill.fore_color.rgb=C(BG)
        rect(s,0,0,.15,H,TEAL,r=0)
        txt(s,.58,.31,11,.30,"T17(1)  ·  CYBER-PHYSICAL SYSTEMS  /  "+chapter,11,TEAL,True)
        txt(s,.58,.84,12.05,.75,title,31,NAVY,True)
        rect(s,.59,1.64,12.05,.018,"CFDEE2",r=0)
        txt(s,.6,7.05,11.8,.25,"MONTANUNIVERSITÄT LEOBEN   ·   FIRST-SEMESTER HANDS-ON WORKSHOP",9,GREY)
        txt(s,12.15,7.05,.56,.25,f"{len(prs.slides):02}",9,TEAL,True)
        return s

    def image(s,name,x=.78,y=1.89,w=11.77,h=4.92):
        path=ART/name
        with Image.open(path) as im: iw,ih=im.size
        scale=min(w/iw,h/ih)
        pw,ph=iw*scale,ih*scale
        s.shapes.add_picture(str(path), Inches(x+(w-pw)/2),Inches(y+(h-ph)/2),width=Inches(pw),height=Inches(ph))

    def diagram(title,name,chapter="HOW IT WORKS"):
        s=slide(title,chapter); image(s,name);return s

    def cardrow(title,pairs,chapter="YOUR CHALLENGE"):
        s=slide(title,chapter)
        count=len(pairs)
        width=(11.95-.18*(count-1))/count
        for i,(h,b) in enumerate(pairs):
            x=.67+i*(width+.18)
            rect(s,x,2.32,width,3.44,PALE)
            txt(s,x+.23,2.64,width-.46,.57,h,22,TEAL,True)
            txt(s,x+.23,3.53,width-.46,1.92,b,19,NAVY)
        return s

    def code_slide(title,code,explain,subtitle,chapter="PYTHON"):
        s=slide(title,chapter)
        rect(s,.67,2.01,7.74,4.42,"E8EEF1")
        txt(s,1.02,2.32,7.10,3.70,code,20,NAVY,False,True)
        rect(s,8.69,2.01,3.96,4.42,PALE)
        txt(s,9,2.35,3.38,.56,subtitle,21,TEAL,True)
        txt(s,9,3.13,3.32,2.85,explain,20,NAVY)
        return s

    def notes(s, text):
        s.notes_slide.notes_text_frame.text = text

    s = diagram("Can a robot run a factory?", "factory-overview.png", "22 OCTOBER 2026")
    notes(s, "3 minutes. Ask: what does a machine need to know before moving a block? Take three answers. Point to sensing, a decision and movement. This is our own teaching schematic, not a photograph of a working sorter. The actual camera model may differ. Autonomous physical sorting is an optional demonstration only after full station acceptance.")

    s = diagram("Our miniature factory", "three-stations.png")
    notes(s, "3 minutes. Point to the actual camera, SO-ARM101 robot, LEGO EV3 conveyor and bins. Identify which parts sense and which move. Students use three pairs. All laptops also have an offline simulator. Source: repository tools/make_assets.py, original teaching diagrams.")

    s = diagram("Observe, decide, act, check", "cps-loop.png")
    notes(s, "4 minutes. Ask students to describe sorting a block by hand. Camera gives information, Python chooses a route, motors act, feedback checks the result. Ask: how would you know that placement actually worked? A command alone does not prove success. Source: repository original schematic.")

    s = diagram("A camera sees pixels", "vision-demo.png")
    notes(s, "3 minutes. Ask which example is red, blue or unknown. Explain that measured color depends on light, background and position. Missing information must produce reject or stop. During activity B compare these schematic examples with a live camera only if it is ready. Source: repository original schematic.")

    s = diagram("Robot joints and gripper", "robot-demo.png")
    notes(s, "3 minutes. Show one joint and the gripper on the actual stationary arm. Explain that reaching a target and avoiding the table both matter. Demonstrate only prechecked supervised motion. Otherwise use simulated robot actions. Source: repository original schematic.")

    s = code_slide("You can program this", 'for number in range(3):\n    print("Hello robot", number + 1)\n\ncolor = "red"\nif color == "red":\n    print("Move left")\nelse:\n    print("Move right")',
        "Predict the output.\n\nChange 3 to 5.\n\nSwap left and right.", "Try a small change")
    notes(s, "4 minutes. Ask for a prediction before running. Explain indentation by pointing to the repeated or conditional lines. The two-way rule here assumes only red and blue. Ask what goes wrong for unknown. Later activities add explicit rejection. Run python exercises/01_python.py. Students need no ROS or hardware access.")

    s = diagram("What can go wrong?", "safety-layout.png")
    notes(s, "3 minutes. Ask for one camera problem, one wrong-position problem and one unexpected-movement problem. Keep hands outside moving equipment. Only the instructor enables physical motion. Stop and ask when behavior is unexpected. Hardware fallback is the simulator. Source: repository original teaching schematic, not a certified physical layout.")

    s = slide("Your challenge")
    image(s, "five-results.png", x=.7, y=1.9, w=8.1, h=4.85)
    txt(s,9.0,2.25,3.6,2.3,"Change a rule.\nTest five objects.\nReject unknown.\nExplain your result.",24,NAVY,True)
    txt(s,9.0,5.05,3.6,1.2,"Work in pairs.\nOne predicts, one types.\nThen swap roles.",19,TEAL)
    notes(s, "2 minutes. Explain success: correct routes and counts for a known five-object sequence, with unknown rejected. Start exercises/01_python.py, then03_vision.py, supervised rotations,04_sort.py and05_visual_sort.py. For visual results open sorting-result.html after running the script. First two independent challenges are reverse rules and counting. Later options: third color, missing readings, new sequences, reliability improvement. Source: repository original diagram.")
    dest=DOCS/"workshop-slides.pptx"
    prs.save(dest)
    return dest


def main():
    make_docx("instructor-guide.docx", "instructor.md")
    make_docx("student-worksheet.docx", "student-worksheet.md", worksheet=True)
    dest = make_slides()
    print("Built:", DOCS / "instructor-guide.docx", DOCS / "student-worksheet.docx", dest, sep="\n")


if __name__ == "__main__":
    main()
