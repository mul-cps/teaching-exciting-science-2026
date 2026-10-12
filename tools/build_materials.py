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


def make_docx(filename, heading, subtitle, hero, sections):
    from docx import Document
    from docx.shared import Inches, Pt, RGBColor
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
    from docx.enum.text import WD_BREAK

    d = Document()
    sec = d.sections[0]
    sec.top_margin, sec.bottom_margin = Inches(.55), Inches(.60)
    sec.left_margin = sec.right_margin = Inches(.78)
    for name, size in (("Normal", 9.2), ("Title", 23), ("Heading 1", 13), ("Heading 2", 11)):
        st = d.styles[name]
        st.font.name, st.font.size = "DejaVu Sans", Pt(size)
        st.font.color.rgb = RGBColor.from_string(NAVY if name != "Normal" else "273C4D")
    d.styles["Normal"].paragraph_format.space_after = Pt(4)
    d.styles["Heading 1"].paragraph_format.space_before = Pt(7)
    d.styles["Heading 1"].paragraph_format.space_after = Pt(2)
    header = sec.header.paragraphs[0]
    header.text = "MONTANUNIVERSITÄT LEOBEN  /  CYBER-PHYSICAL SYSTEMS"
    header.runs[0].font.name = "DejaVu Sans"
    header.runs[0].font.color.rgb = RGBColor.from_string(TEAL)
    header.runs[0].font.size = Pt(7.5)
    p = d.add_paragraph(style="Title")
    p.add_run(heading)
    p = d.add_paragraph(subtitle)
    p.runs[0].font.color.rgb = RGBColor.from_string(GREY)
    p.runs[0].font.size = Pt(10)
    pict = d.add_paragraph()
    pict.alignment = WD_ALIGN_PARAGRAPH.CENTER
    pict.paragraph_format.space_after = Pt(2)
    pict.add_run().add_picture(str(ART / hero), width=Inches(5.9))
    for title, elements in sections:
        if title == "__PAGE_BREAK__":
            d.add_page_break()
            continue
        d.add_heading(title, 1)
        if elements and isinstance(elements[0], tuple):
            table = d.add_table(rows=1, cols=len(elements[0]))
            table.style = "Light Shading Accent 1"
            table.alignment = WD_TABLE_ALIGNMENT.CENTER
            for j, content in enumerate(elements[0]):
                table.rows[0].cells[j].text = str(content)
            for values in elements[1:]:
                row = table.add_row()
                for j, content in enumerate(values):
                    row.cells[j].text = str(content)
                    row.cells[j].vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            for row in table.rows:
                for cell in row.cells:
                    for para in cell.paragraphs:
                        for run in para.runs:
                            run.font.size = Pt(8)
        else:
            for element in elements:
                if isinstance(element, str) and element.startswith("FIGURE:"):
                    p = d.add_paragraph()
                    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                    p.paragraph_format.space_after = Pt(1)
                    p.add_run().add_picture(str(ART / element[7:]), width=Inches(5.9))
                elif isinstance(element, str) and element.startswith("CODE:"):
                    p = d.add_paragraph()
                    p.paragraph_format.space_after = Pt(3)
                    p.paragraph_format.left_indent = Inches(.22)
                    for line in element[5:].split("\n"):
                        r = p.add_run(line)
                        r.font.name, r.font.size = "DejaVu Sans Mono", Pt(8)
                        r.font.color.rgb = RGBColor.from_string(TEAL)
                        r.add_break()
                else:
                    d.add_paragraph(str(element), style="List Bullet" if element.startswith("• ") else None)
    footer = sec.footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    footer.text = "T17(1) | Exciting Science 2026  ·  Workshop draft"
    footer.runs[0].font.color.rgb = RGBColor.from_string(GREY)
    footer.runs[0].font.size = Pt(7)
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

    # 1-4. One goal and one visual language throughout. All art is local, editable SVG source.
    diagram("Programmiert eure eigene Smart Factory", "factory-overview.png", "MISSION")
    cardrow("Unser Ziel / Today's goal", [("1 · SENSE", "Camera sees a red or blue token."),("2 · DECIDE", "Python chooses left, right, or reject."),("3 · ACT", "Robot moves only when the belt is stopped.")],"MISSION")
    diagram("Was macht das System cyber-physisch?", "cps-loop.png", "CONCEPTS")
    cardrow("Physisch + Digital + Rückmeldung",[("PHYSICAL", "Conveyor, robot, blocks"),("DIGITAL", "Python, colors, sorting rules"),("FEEDBACK", "Stop, check, then repeat")],"CONCEPTS")
    diagram("Sicherheit zuerst / Safety first", "safety-layout.png", "SAFETY")
    diagram("A · LeRobot: Vormachen und Nachmachen", "robot-demo.png", "STATION A")
    diagram("B · Kamera: Einen Gegenstand erkennen", "vision-roi.png", "STATION B")
    diagram("C · LEGO EV3: Bewegen und stoppen", "belt-demo.png", "STATION C")
    code_slide("Python in drei Minuten",'for number in range(3):\n    print("Hello robot", number + 1)',"Change 3 to 5.\n\nWhat happens?\n\nTry it together.","Exercise 01", "PYTHON")
    diagram("Drei Stationen · Three stations", "three-stations.png", "STUDENT ROTATION")
    code_slide("Station A · Erste Roboterbefehle",'from workshop import SimRobot\n\nrobot = SimRobot()\nrobot.sort("left")\nrobot.sort("right")\nprint(robot.actions)',"Simulation first.\n\nTry swapping left and right.\n\nReal robot: supervised demo only.","Exercise 02", "STATION A")
    code_slide("Station B · Die Kamera sagt eine Farbe",'from workshop import SimCamera\n\ncamera = SimCamera(\n    ["red", "blue", "unknown"]\n)\nprint(camera.detect())',"What if the result is unknown?\n\nUnknown is not blue.\n\nStop or reject.","Exercise 03", "STATION B")
    code_slide("Station C · Das Band sicher stoppen",'from workshop import SimBelt\n\nbelt = SimBelt()\nbelt.advance()\nbelt.stop()',"Never reach into a moving belt.\n\nPhysical EV3 control still needs an instructor bench test.","Simulated belt", "STATION C")
    diagram("Die Entscheidung: links, rechts oder Fehler", "sorting-rules.png", "PYTHON")
    code_slide("Euer Sortierprogramm",'from workshop import Factory, SimBelt\nfrom workshop import SimCamera, SimRobot\n\nf = Factory(SimBelt(),\n    SimCamera(), SimRobot())\nfor _ in range(5):\n    f.one_cycle({"red": "left",\n                 "blue": "right"})',"Five cycles.\n\nRed → left.\nBlue → right.\nUnknown → reject.\n\nChange the rules!","Exercise 04", "PYTHON")
    diagram("Der sichere Ablauf: Stop, erkennen, sortieren", "safe-sort-cycle.png", "INTEGRATION")
    diagram("Der Testlauf: 5 Objekte", "five-results.png", "INTEGRATION")
    cardrow("Team-Challenges / Try it",[("EASY", "Swap red and blue.\nPredict the results."),("MEDIUM", "Count left / right / reject.\nWhy five objects?"),("BONUS", "Test a real webcam if the instructor approves.")],"TRY & IMPROVE")
    diagram("Wenn Hardware fehlt: Simulation läuft weiter", "fallback.png", "ROBUSTNESS")
    cardrow("Was nehmt ihr mit? / Recap",[("SENSOR", "What does the camera measure?"),("PYTHON", "What rule makes the decision?"),("ACTUATOR", "What actually moves?"),("SAFETY", "What should stop the system?")],"REFLECTION")
    dest=DOCS/"workshop-slides.pptx"
    prs.save(dest)
    return dest


def main():
    make_docx("instructor-guide.docx", "CPS Candy Factory", "T17(1) · Instructor guide · 4 hours · 6 students", "factory-overview.png", [
        ("Learning objective", ["Students explore sensing → decision → action → verification using a safe Python simulation and optional supervised hardware stations.", "All students must be able to change an if rule and explain why the belt must stop before sorting."]),
        ("Four-hour schedule", [("Time", "Block", "Outcome"),("00:00–00:45", "CPS, Python, safety & demo", "Understand"),("00:45–01:45", "3 × 20 min stations", "Try hardware/simulation"),("01:45–01:55", "Break", ""),("01:55–02:20", "Make a sorting rule", "Write if / else"),("02:20–03:05", "Five-cycle sorter", "Test"),("03:05–03:35", "Team challenges", "Improve"),("03:35–04:00", "Demos & reflection", "Explain CPS")]),
        ("Group rotation", ["Team 1: A → B → C; Team 2: B → C → A; Team 3: C → A → B. Two students per team; rotate operator and observer every 10 minutes."]),
        ("__PAGE_BREAK__", []),
        ("Instructor preflight", ["1. Install Python and run python -m workshop on each student laptop; instructor runs pytest before class.","2. Test all camera, EV3 and robot hardware before enabling motion. See docs/acceptance.md.","3. Secure table, enforce exclusion zone, confirm accessible power disconnect.","4. Prepare 20–30 mm colored blocks, a stop guide, and fixed lighting.","5. Keep paper copies and offline simulation available for every team."]),
        ("Safe sorting cycle", ["FIGURE:safe-sort-cycle.png","Missing images, unknown detections, or obstructions never trigger an unvalidated motion."]),
        ("Minimal reliable demo", ["CODE:python -m workshop\npython exercises/01_python.py\npython exercises/02_robot.py\npython exercises/03_vision.py\npython exercises/04_sort.py"]),
        ("Not yet demonstrated physically", ["HERO10 transport, EV3 movement, SO-ARM101 teleoperation and autonomous gripping must be bench-validated. The PowerPoint shows schematics, not photos of a completed sorter."])
    ])
    make_docx("student-worksheet.docx", "Unsere Smart Factory", "T17(1) · Student worksheet · Team: __________ · Names: _______________________", "cps-loop.png", [
        ("Your mission", ["Sensor → Python → actuator → feedback. Sort red and blue tokens; unknown results must not be guessed."]),
        ("1 · Hello robot", ["CODE:python exercises/01_python.py","Change range(3) to range(5). Write down the output: ___________________________"]),
        ("2 · Plan robot actions", ["CODE:python exercises/02_robot.py","Swap left and right. What changed? ________________________________________"]),
        ("3 · Camera decision", ["CODE:python exercises/03_vision.py","Try red / blue / unknown. What must happen with unknown? ____________________"]),
        ("__PAGE_BREAK__", []),
        ("4 · The five-object challenge", ["FIGURE:five-results.png","CODE:python exercises/04_sort.py","Swap red and blue destinations. Predict the counts before running.","Left: ______ Right: ______ Reject: ______", "Bonus: use a real webcam only after the instructor has tested and enabled it."]),
        ("What is cyber-physical here?", ["Sensor: _____________________  Decision: _____________________", "Actuator: __________________  Safety rule: ___________________"]),
        ("Safety", ["Hands away from the moving robot and belt. If motion, camera, or an object behaves unexpectedly, stop and ask the instructor."]),
    ])
    dest=make_slides()
    print("Built:", DOCS/"instructor-guide.docx", DOCS/"student-worksheet.docx", dest,sep="\n")


if __name__=="__main__":
    main()
