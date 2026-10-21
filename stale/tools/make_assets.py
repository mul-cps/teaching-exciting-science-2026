"""Generate the workshop's original diagrams (SVG + PNG) for print and slides.

Run: python stale/tools/make_assets.py  (requires optional cairosvg for PNG output).
All illustrations are explanatory schematics, not photos or CAD of the hardware.
"""
from pathlib import Path
from xml.sax.saxutils import escape

OUT = Path(__file__).resolve().parents[1] / "docs" / "assets"
OUT.mkdir(parents=True, exist_ok=True)

NAVY, TEAL, GOLD, GREY = '#17354B', '#197F80', '#BF8D42', '#607383'
BG, RED, BLUE, GREEN = '#F6FAFB', '#DC6658', '#609BD0', '#7AB895'


def text(x, y, s, size=22, color=NAVY, bold=False, anchor="start"):
    return f'<text x="{x}" y="{y}" fill="{color}" font-family="DejaVu Sans,Arial,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}">{escape(s)}</text>'


def roundrect(x,y,w,h,color,stroke='none',r=20,sw=2):
    return f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{color}" stroke="{stroke}" stroke-width="{sw}"/>'


def line(x1,y1,x2,y2,color=NAVY,sw=5,dash=''):
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{sw}" stroke-linecap="round" {f"stroke-dasharray={chr(34)+dash+chr(34)}" if dash else ""}/>'


def arrow(x1,y1,x2,y2,color=TEAL,sw=5):
    return line(x1,y1,x2,y2,color,sw)+f'<polygon points="{x2},{y2} {x2-15},{y2-10} {x2-15},{y2+10}" fill="{color}" transform="rotate({__import__("math").degrees(__import__("math").atan2(y2-y1,x2-x1))},{x2},{y2})"/>'


def wrap(width,height,parts):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">'
            +f'<rect width="{width}" height="{height}" fill="{BG}"/>' + ''.join(parts)+'</svg>')


def conveyor(x,y,w=560):
    p=[roundrect(x,y,w,128,'#DCE8EE','#91A6B2',44,5),roundrect(x+15,y+17,w-30,94,'#A9BCCA','#8197A6',35,3)]
    for i in range(7):
        xx=x+52+i*(w-95)/6
        p.append(line(xx,y+30,xx,y+98,'#8DA1AF',3))
    p.append(f'<circle cx="{x+w-12}" cy="{y+64}" r="9" fill="{NAVY}"/>')
    return ''.join(p)


def arm(x,y,k=1):
    # schematic segmented SO-ARM101-style manipulator, not exact geometry
    q=lambda a,b:f'{x+a*k:.1f},{y+b*k:.1f}'
    p=[roundrect(x-64*k,y+6*k,136*k,35*k,'#687B8A','#41566A',9*k,4)]
    joints=[(0,0),(-23,-85),(-132,-157),(-193,-116)]
    for (ax,ay),(bx,by) in zip(joints,joints[1:]):
        p.append(line(x+ax*k,y+ay*k,x+bx*k,y+by*k,'#627C90',17*k))
        p.append(line(x+ax*k,y+ay*k,x+bx*k,y+by*k,'#A6BFCD',6*k))
    for a,b in joints[:-1]:p.append(f'<circle cx="{x+a*k}" cy="{y+b*k}" r="{13*k}" fill="#DCE8EE" stroke="#526E82" stroke-width="{5*k}"/>')
    p.append(line(x-193*k,y-116*k,x-207*k,y-101*k,'#4B7183',7*k));p.append(line(x-193*k,y-116*k,x-179*k,y-101*k,'#4B7183',7*k))
    return ''.join(p)


def camera(x,y,scale=1):
    return (roundrect(x,y,110*scale,74*scale,'#293A45','#152C3A',10*scale,4)
        +f'<circle cx="{x+69*scale}" cy="{y+37*scale}" r="{25*scale}" fill="#0C2431" stroke="#6D8997" stroke-width="{6*scale}"/>'
        +f'<circle cx="{x+69*scale}" cy="{y+37*scale}" r="{10*scale}" fill="#73A7BA"/>'
        +roundrect(x+9*scale,y+8*scale,30*scale,17*scale,'#87ABB7','none',3*scale))


def overview():
    p=[text(60,76,'A SMART SORTING FACTORY',40,TEAL,True),text(60,122,'One object · one camera · one decision · one safe movement',23,GREY)]
    p.append(conveyor(80,342,670))
    for x,c in [(210,RED),(365,BLUE),(512,RED)]:
        p.append(roundrect(x,368,54,55,c,'#FFFFFF',11,3))
    p.append(roundrect(702,367,54,55,GOLD,'#FFFFFF',10,3));p.append(text(723,457,'PICK',17,NAVY,True))
    p.append(arm(1120,471,1.27));p.append(text(1000,617,'SO-ARM101 follower',24,NAVY,True))
    for x,c,t in [(1300,RED,'LEFT · RED'),(1446,BLUE,'RIGHT · BLUE')]:
        p.append(roundrect(x,400,116,110,'#E1EAEF','#8EA6B1',11,3))
        p.append(roundrect(x+12,416,92,14,c,'none',5));p.append(text(x+58,470,t,13,NAVY,True,'middle'))
    p.append(line(608,195,608,325,TEAL,5,'11 11'))
    p.append(camera(553,181,1.0));p.append(line(608,180,608,130,GREY,8))
    p.append(line(550,151,675,151,GREY,8));p.append(line(608,243,608,329,TEAL,3,'8 7'))
    p.append(text(686,209,'Camera',22,NAVY,True));p.append(text(686,240,'stationary overhead view',17,GREY))
    p.append(roundrect(50,578,475,98,'#E4EFF0','none',18));p.append(text(77,616,'LEGO EV3',24,TEAL,True));p.append(text(77,652,'Motor drives the conveyor belt',18,NAVY))
    p.append(roundrect(702,575,236,102,'#EBEAE3','#CFC7AB',18,3));p.append(text(820,611,'PYTHON',24,NAVY,True,'middle'));p.append(text(820,644,'red → left',18,NAVY,False,'middle'))
    p.append(arrow(750,407,878,407,TEAL,6))
    p.append(text(810,392,'stop',17,TEAL,True,'middle'))
    p.append(text(65,745,'SCHEMATIC ONLY · Test all positions, gripper clearance and camera field of view on real hardware.',18,GREY))
    return wrap(1620,785,p)


def cps_loop():
    labels=[('01','SENSE','Camera observes the token',TEAL),('02','DECIDE','Python checks its color',GOLD),('03','ACT','Robot and EV3 move',BLUE),('04','VERIFY','Stop if something is wrong',GREEN)]
    p=[text(64,75,'THE CPS LOOP',39,TEAL,True)]
    for i,(num,heading,detail,color) in enumerate(labels):
        x=57+i*390
        p.append(roundrect(x,139,340,205,'#FFFFFF','#D5E3E8',24,3))
        p.append(roundrect(x+22,169,62,52,color,'none',15))
        p.append(text(x+53,205,num,25,'#FFFFFF',True,'middle'))
        p.append(text(x+104,203,heading,28,NAVY,True))
        p.append(text(x+22,275,detail,19,GREY))
        if i<3:p.append(arrow(x+344,241,x+382,241,TEAL,4))
    p.append(line(1455,380,1455,424,GREY,4));p.append(line(1455,424,225,424,GREY,4));p.append(line(225,424,225,372,GREY,4))
    p.append(text(790,463,'repeat only after the previous action has completed',20,GREY,False,'middle'))
    return wrap(1620,500,p)


def stations():
    p=[text(55,76,'THREE HANDS-ON STATIONS',39,TEAL,True)]
    cards=[('A','ROBOT','Teach and replay',TEAL),('B','CAMERA','Recognize red / blue',BLUE),('C','CONVEYOR','Move and stop',GOLD)]
    for i,(letter,name,detail,color) in enumerate(cards):
        x=55+i*520
        p.append(roundrect(x,134,480,330,'#FFFFFF','#DCE9EB',24,3))
        p.append(roundrect(x+30,166,72,62,color,'none',18));p.append(text(x+66,209,33,'#FFFFFF',True,'middle') if False else text(x+66,209,letter,32,'#FFFFFF',True,'middle'))
        p.append(text(x+123,208,name,31,NAVY,True));p.append(text(x+36,425,detail,22,GREY))
        if i==0:p.append(arm(x+315,350,.67))
        if i==1:
            p.append(camera(x+177,260,.8));p.append(roundrect(x+319,297,62,61,RED,'#FFF',10,3));p.append(roundrect(x+387,297,62,61,BLUE,'#FFF',10,3));p.append(line(x+266,298,x+312,324,color,3,'6 5'))
        if i==2:p.append(conveyor(x+87,270,328));p.append(roundrect(x+219,291,46,46,RED,'#FFF',8,2))
    return wrap(1620,520,p)


def vision():
    p=[text(60,73,'CAMERA PERCEPTION · SIMULATED VIEW',38,TEAL,True)]
    p.append(roundrect(50,110,945,570,'#273841','#273841',25,3))
    p.append(roundrect(101,145,846,500,'#374F5B','none',20))
    for x,c,name in [(220,RED,'red'),(610,BLUE,'blue')]:
        p.append(roundrect(x+30,310,140,137,c,'#FFFFFF',21,4))
        p.append(f'<rect x="{x}" y="280" width="200" height="202" rx="15" fill="none" stroke="#ECF6F4" stroke-width="6"/>')
        p.append(roundrect(x,242,200,44,TEAL,'none',9))
        p.append(text(x+100,271,name+' token',19,'#FFFFFF',True,'middle'))
    p.append(roundrect(1050,110,505,568,'#E4EFF0','none',25))
    p.append(text(1095,171,'PYTHON RECEIVES',24,TEAL,True))
    p.append(text(1095,218,'color = "red"',26,NAVY,True))
    p.append(text(1095,292,'if color == "red":',23,NAVY))
    p.append(text(1135,330,'sort_left()',23,NAVY))
    p.append(text(1095,381,'elif color == "blue":',20,NAVY))
    p.append(text(1135,415,'sort_right()',21,NAVY))
    p.append(text(1095,466,'else:',22,NAVY))
    p.append(text(1135,502,'reject_unknown()',21,NAVY))
    p.append(text(1090,589,'No GPU required for',20,GREY))
    p.append(text(1090,619,'CPU color sorting.',20,GREY))
    return wrap(1620,718,p)


def safety():
    p=[text(60,76,'SAFE BENCH LAYOUT · TOP VIEW',39,TEAL,True)]
    p.append(roundrect(70,135,970,515,'#F5F1E9','#CABDAB',22,5))
    p.append(conveyor(129,344,543))
    p.append(arm(868,444,.88));p.append(text(758,572,'ARM WORKSPACE',21,RED,True))
    p.append(f'<circle cx="{860}" cy="{399}" r="{201}" fill="{RED}" fill-opacity=".07" stroke="{RED}" stroke-width="4" stroke-dasharray="13 11"/>')
    p.append(roundrect(562,378,52,49,BLUE,'#FFFFFF',9,3))
    p.append(text(563,454,'FIXED',16,TEAL,True));p.append(text(563,475,'PICKUP',16,TEAL,True))
    p.append(camera(412,170,.85));p.append(line(458,229,458,311,TEAL,3,'8 6'))
    p.append(text(525,197,'Camera above belt',21,GREY))
    p.append(roundrect(1091,139,459,505,'#E4EFF0','none',25))
    for i,(k,v) in enumerate([('1','Stop conveyor before reaching'),('2','Do not enter the marked area'),('3','Keep a reachable power cutoff'),('4','Stop on missing camera frames')]):
        p.append(roundrect(1119,195+i*100,46,44,TEAL,'none',11))
        p.append(text(1142,226+i*100,k,22,'#FFFFFF',True,'middle'))
        p.append(text(1183,222+i*100,v,18,NAVY))
    p.append(text(62,724,'SCHEMATIC ONLY · Instructor must verify the actual arm sweep and emergency-stop arrangement.',18,GREY))
    return wrap(1620,760,p)



def sorting_rules():
    """Branching in Python: one sensor label, three safe outcomes."""
    p=[text(62,76,'HOW PYTHON MAKES A DECISION',39,TEAL,True)]
    p.append(camera(72,180,.88))
    p.append(text(195,205,'camera.detect_color()',25,NAVY,True))
    p.append(arrow(410,217,545,217,TEAL,5))
    p.append(roundrect(552,147,330,143,'#E4EFF0','#9CB2BC',20,3))
    p.append(text(718,195,'if / elif / else',29,NAVY,True,'middle'))
    p.append(text(718,238,'choose one outcome',19,GREY,False,'middle'))
    for i,(c,header,desc) in enumerate([(RED,'RED','SORT LEFT'),(BLUE,'BLUE','SORT RIGHT'),(GOLD,'UNKNOWN','DO NOT GUESS')]):
        yy=60+i*173
        p.append(line(889,215,952,215,GREY,3) if i==0 else '')
        p.append(line(934,215,934,yy+67,GREY,3))
        p.append(arrow(934,yy+67,988,yy+67,TEAL,4))
        p.append(roundrect(1000,yy,545,132,'#FFFFFF','#D8E3E9',20,3))
        p.append(roundrect(1020,yy+26,65,65,c,'#FFFFFF',10,3))
        p.append(text(1113,yy+55,header,25,NAVY,True))
        p.append(text(1113,yy+89,desc,20,GREY))
    p.append(text(62,596,'An unknown result is a valid result. Never move blindly when sensing fails.',20,GREY))
    return wrap(1620,645,p)


def stop_pick_place():
    """An easy-to-understand state machine for stopped-belt sorting."""
    p=[text(62,76,'STOP · PICK · PLACE · REPEAT',39,TEAL,True)]
    states=[('1','ADVANCE',BLUE),('2','STOP',GOLD),('3','DETECT',TEAL),('4','SORT',GREEN)]
    for i,(num,title,c) in enumerate(states):
        x=50+i*393
        p.append(roundrect(x,136,356,346,'#FFFFFF','#D5E3E8',22,3))
        p.append(roundrect(x+22,158,68,55,c,'none',15))
        p.append(text(x+56,197,num,28,'#FFFFFF',True,'middle'))
        p.append(text(x+109,197,title,27,NAVY,True))
        if i==0:
            p.append(conveyor(x+42,282,260));p.append(roundrect(x+90,300,40,40,RED,'#FFF',7,2))
        elif i==1:
            p.append(conveyor(x+42,282,260));p.append(roundrect(x+172,300,40,40,RED,'#FFF',7,2))
            p.append(text(x+172,264,'!',29,RED,True))
        elif i==2:
            p.append(camera(x+112,270,.85));p.append(line(x+160,339,x+160,381,TEAL,3,'7 5'))
            p.append(roundrect(x+151,386,45,45,RED,'#FFF',7,2))
        else:
            p.append(arm(x+255,392,.57));p.append(roundrect(x+40,377,90,67,'#DCE8EE','#809BA8',10,2))
            p.append(roundrect(x+58,385,52,12,RED,'none',4))
        p.append(text(x+34,457,['Move one token','Belt must be idle','Look at fixed location','Only validated motions'][i],18,GREY))
        if i<3:p.append(arrow(x+360,301,x+384,301,TEAL,4))
    p.append(text(810,552,'If a step fails: stop and ask the instructor.',22,RED,True,'middle'))
    return wrap(1620,592,p)



def robot_demo():
    p=[text(62,73,'LEADER → FOLLOWER',37,TEAL,True),text(62,119,'Students guide a robot; the instructor supervises real motion.',22,GREY)]
    p.append(roundrect(60,160,690,440,'#FFFFFF','#D6E5E9',26,3))
    p.append(arm(545,460,1.10));p.append(text(102,218,'LEADER',28,TEAL,True))
    p.append(text(102,254,'Move a joint by hand',20,GREY))
    p.append(roundrect(810,160,740,440,'#FFFFFF','#D6E5E9',26,3))
    p.append(arm(1355,460,1.10));p.append(text(853,218,'FOLLOWER',28,TEAL,True))
    p.append(text(853,254,'Repeats the taught motion',20,GREY))
    p.append(arrow(735,400,804,400,GOLD,7))
    p.append(roundrect(290,634,1060,83,'#E4EFF0','none',18));p.append(text(820,684,'Hardware demo ONLY after calibration and safe reach test.',23,NAVY,True,'middle'))
    return wrap(1620,760,p)


def belt_demo():
    p=[text(60,77,'MOVE → STOP',39,TEAL,True),text(60,123,'One safe advance brings the next block to a fixed pickup point.',21,GREY)]
    p.append(conveyor(100,275,1150))
    for x,c in [(272,RED),(525,BLUE),(775,RED)]:p.append(roundrect(x,305,66,67,c,'#FFFFFF',12,4))
    p.append(roundrect(1065,300,66,67,BLUE,'#FFFFFF',12,4))
    p.append(line(1100,241,1100,456,TEAL,4,'11 10'))
    p.append(text(1100,227,'PICKUP',22,TEAL,True,'middle'))
    p.append(arrow(340,477,680,477,TEAL,6));p.append(text(508,514,'ADVANCE',19,TEAL,True,'middle'))
    p.append(roundrect(1316,267,210,151,'#E4EFF0','#B9D0D4',20,2))
    p.append(text(1421,313,'BELT',28,TEAL,True,'middle'))
    p.append(text(1421,349,'stop()',29,NAVY,True,'middle'))
    p.append(text(60,626,'Why stop? A moving object is harder to grasp safely and repeatably.',24,NAVY))
    p.append(text(60,693,'Diagram illustrates the logic; EV3 motor control is not yet bench validated.',17,GREY))
    return wrap(1620,760,p)


def vision_roi():
    p=[text(60,76,'ONE CAMERA · ONE FIXED REGION',37,TEAL,True)]
    p.append(camera(96,164,1.15));p.append(line(162,250,162,338,TEAL,4,'8 6'))
    p.append(text(255,206,'GoPro HERO10 or USB webcam',24,NAVY,True));p.append(text(255,245,'Use standard CPU color recognition first',20,GREY))
    p.append(roundrect(70,345,880,330,'#293F4B','none',25))
    p.append(roundrect(116,370,788,280,'#728A98','none',19))
    p.append(roundrect(419,414,187,188,'none','#E7FCF6',15,6))
    p.append(roundrect(476,459,74,75,RED,'#FFFFFF',12,3))
    p.append(roundrect(150,462,66,66,BLUE,'#FFFFFF',10,3))
    p.append(text(512,635,'FIXED PICKUP ROI',20,'#FFFFFF',True,'middle'))
    p.append(roundrect(1035,333,515,344,'#E4EFF0','none',25))
    p.append(text(1084,391,'Python receives',26,TEAL,True))
    p.append(text(1084,449,'"red"',40,NAVY,True))
    p.append(text(1084,513,'if red → left',23,NAVY,True))
    p.append(text(1084,557,'if blue → right',23,NAVY,True))
    p.append(text(1084,600,'otherwise → reject',23,NAVY,True))
    p.append(text(73,732,'Not real detections; schematic view. Test thresholds with actual lighting.',17,GREY))
    return wrap(1620,760,p)


def five_results():
    p=[text(58,77,'TEST FIVE OBJECTS',39,TEAL,True)]
    p.append(text(58,122,'Simulation input: red, blue, red, unknown, blue',22,GREY))
    for i,(color,out) in enumerate([(RED,'LEFT'),(BLUE,'RIGHT'),(RED,'LEFT'),(GOLD,'REJECT'),(BLUE,'RIGHT')]):
        x=50+i*316
        p.append(roundrect(x,190,291,335,'#FFFFFF','#D8E4E8',24,3))
        p.append(text(x+145,245,str(i+1),24,TEAL,True,'middle'))
        p.append(roundrect(x+105,278,82,84,color,'#FFFFFF',15,4))
        p.append(text(x+145,428,out,29,NAVY,True,'middle'))
    p.append(roundrect(147,566,420,113,'#E4EFF0','none',22));p.append(text(355,614,'LEFT',23,TEAL,True,'middle'));p.append(text(355,655,'2 blocks',25,NAVY,True,'middle'))
    p.append(roundrect(599,566,420,113,'#E4EFF0','none',22));p.append(text(810,614,'RIGHT',23,TEAL,True,'middle'));p.append(text(810,655,'2 blocks',25,NAVY,True,'middle'))
    p.append(roundrect(1051,566,420,113,'#F5EADF','none',22));p.append(text(1261,614,'REJECT',23,TEAL,True,'middle'));p.append(text(1261,655,'1 block',25,NAVY,True,'middle'))
    return wrap(1620,740,p)


def fallback():
    p=[text(60,78,'A SAFE FALLBACK IS STILL A WORKING CPS',37,TEAL,True)]
    pairs=[('1','HARDWARE READY?','Instructor verifies real setup',TEAL),('2','IF NOT','Switch to offline simulator',GOLD),('3','LEARNING CONTINUES','All exercises work on CPU',BLUE)]
    for i,(n,h,d,c) in enumerate(pairs):
        x=55+i*524
        p.append(roundrect(x,183,484,384,'#FFFFFF','#D7E5E8',25,3))
        p.append(roundrect(x+35,220,67,66,c,'none',16));p.append(text(x+68,264,n,32,'#FFFFFF',True,'middle'))
        p.append(text(x+35,358,h,28,NAVY,True));p.append(text(x+35,407,d,20,GREY))
        if i<2:p.append(arrow(x+482,376,x+518,376,TEAL,4))
    p.append(roundrect(326,617,960,85,'#E4EFF0','none',18));p.append(text(807,671,'No cloud account · No GPU · No installation during class',22,NAVY,True,'middle'))
    return wrap(1620,760,p)

if __name__=='__main__':
    assets={'factory-overview':overview(), 'cps-loop':cps_loop(), 'three-stations':stations(), 'vision-demo':vision(), 'safety-layout':safety(), 'sorting-rules':sorting_rules(), 'safe-sort-cycle':stop_pick_place(), 'robot-demo':robot_demo(), 'belt-demo':belt_demo(), 'vision-roi':vision_roi(), 'five-results':five_results(), 'fallback':fallback()}
    try:
        import cairosvg
    except ImportError:
        cairosvg=None
    for name,source in assets.items():
        path=OUT/(name+'.svg');path.write_text(source,encoding='utf-8')
        if cairosvg:
            cairosvg.svg2png(bytestring=source.encode(),write_to=str(OUT/(name+'.png')))
        print(path.name,'+ PNG' if cairosvg else '(PNG needs cairosvg)')
