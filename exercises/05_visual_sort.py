"""Edit rules, run this file, then open sorting-result.html in any browser."""
import argparse
import html
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import Factory, SimBelt, SimCamera, SimRobot

# Student area: predict the result, then change the mapping or object sequence.
rules = {"red": "left", "blue": "right"}
objects = ["red", "blue", "red", "unknown", "blue"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("sorting-result.html"))
    args = parser.parse_args()
    factory = Factory(SimBelt(), SimCamera(objects), SimRobot())
    cards = []
    for index, color in enumerate(objects):
        destination = factory.one_cycle(rules)
        paint = color if color in ("red", "blue", "green", "yellow") else "#777"
        end = {"left": "-32vw", "right": "32vw", "reject": "0px"}[destination]
        cards.append(f'<div class="object" style="--paint:{paint};--end:{end};--drop:{100 + factory.counts[destination] * 38}px;--delay:{index * 1.5}s">'
                     f'{html.escape(str(color))} → {destination}</div>')
    counts = " · ".join(f"{key}: {value}" for key, value in factory.counts.items())
    document = '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Your sorting factory</title>
<style>
body{font:18px system-ui;margin:2rem;background:#f2f5fa;color:#142135;text-align:center}
.scene{position:relative;height:390px;max-width:1100px;margin:auto;background:#dde5ec;border-radius:16px}
.bins{position:absolute;bottom:15px;width:100%;display:flex;justify-content:space-around}
.bin{border:3px solid #344b63;border-top:0;padding:20px 10px;min-width:18%;background:white}
.object{position:absolute;left:calc(50% - 70px);top:15px;width:140px;padding:8px 0;background:var(--paint);
color:white;border:2px solid #142135;border-radius:8px;opacity:0;animation:sort 1.3s var(--delay) forwards}
@keyframes sort{0%{opacity:1;transform:translate(0,0)}30%{opacity:1;transform:translate(0,80px)}
100%{opacity:1;transform:translate(var(--end),var(--drop))}}
@media(prefers-reduced-motion:reduce){.object{animation-duration:0.01s}}
</style><h1>Your sorting factory</h1><p>Edit <b>rules</b> and <b>objects</b> in 05_visual_sort.py, run again, then refresh.</p>
<p>Observe → Decide → Act → Check. Simulation only; no hardware is connected.</p>
<div class="scene"><div class="bins"><div class="bin">LEFT</div><div class="bin">REJECT</div><div class="bin">RIGHT</div></div>
'''
    document += "\n".join(cards) + f'</div><h2>{counts}</h2><p>Reload to replay. Did your prediction match?</p></html>'
    args.output.write_text(document, encoding="utf-8")
    print("Open in your browser:", args.output.resolve())


if __name__ == "__main__":
    main()
