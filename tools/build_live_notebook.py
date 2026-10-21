"""Generate a short subscriber-only robot station notebook with safe demo default."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


def build():
    cells = []
    def md(text): cells.append({'cell_type':'markdown','metadata':{},'source':text.splitlines(True)})
    def code(text): cells.append({'cell_type':'code','metadata':{},'source':text.splitlines(True),'execution_count':None,'outputs':[]})
    md('# Robot joint detective\n\n22 October 2026 · Robot and twin station · 15 minutes\n\nA **joint** is a movable connection. Its angle is a number measured from a calibrated reference. You will read numbers, predict a change and compare a graph with the robot and its digital twin. A **digital twin** is a computer model updated from measurements.\n\nThis notebook only observes. The instructor operates **enable, mode and stop in the RViz operations panel**. Keep clear of the moving workspace; do not move an enabled follower by hand. No notebook cell sends movement or calls a ROS service. Start in labelled invented demo data; LIVE is a separately prepared instructor launch. A successful graph is not evidence that a physical placement succeeded.')
    md('## 1 Open and read 3 minutes\n\nMake a team copy with File → Save Notebook As. Click each short code cell and press Shift+Enter. Run from the top after restarting the kernel. The setup below finds the supplied helper; it installs nothing. **Predict:** will all joint angles be equal?')
    code('from pathlib import Path\nimport sys, os\nroot = Path.cwd()\nif not (root / "workshop").is_dir():\n    root = root.parent\nif str(root) not in sys.path:\n    sys.path.insert(0, str(root))\nfrom workshop.live_observer import observe, plot_samples, JOINTS\nLIVE = os.environ.get("WORKSHOP_LIVE") == "1"\nprint("LIVE topic observation" if LIVE else "DEMO: invented data, no robot measurements")')
    md('Ask the instructor to hold the leader still during this first observation. Collection stops after three seconds. The helper starts fresh each time; missing data are shown explicitly and never replaced with demo data.')
    code('report = observe(live=LIVE, duration=3)\nprint(report["mode"])\nprint(report["samples"][-1] if report["samples"] else report.get("error", "No samples"))')
    md('**Check:** look for `fresh`, `stale` or `not_running`. Fresh requires recent receipt and source timestamps. Stale means too old; not_running means no message received, which may also mean wrong ROS discovery settings. Never interpret a missing angle as zero. Joint source age is the ROS header age; the follower header marks publication, not independent motor acquisition. Teleop status may be an old latched message when disabled. Its old state is not permission to move. Ask the instructor to investigate. Do not install ROS in this Jupyter environment.')
    md('## 2 Pick one joint 3 minutes\n\nThe five body joints report angles in radians; the helper converts them to degrees. One full turn is 360 degrees or about 6.28 radians. The gripper value in JointState is a visual estimate, so we leave it out. **Predict:** which line will be higher? Change only the joint name to another name from the printed list, then run again.')
    code('print(JOINTS)\nJOINT = "shoulder_pan"\nplot_samples(report, JOINT)')
    md('## 3 Predict one small change 4 minutes\n\nIn **relative mode** the follower repeats the leader change from their own start positions: follower start + leader now − leader start. In **absolute mode** the calibrated leader angle is the requested follower angle. A requested angle is not a measured result.\n\nThese next values are paper predictions only. They never reach the robot. **Predict:** starting at leader 10°, follower 30°, then moving the leader to 15°, what will relative mode request? Change only `LEADER_NOW` and rerun.')
    code('LEADER_START = 10\nFOLLOWER_START = 30\nLEADER_NOW = 15\nrelative_prediction = FOLLOWER_START + LEADER_NOW - LEADER_START\nabsolute_prediction = LEADER_NOW\nprint("Relative prediction:", relative_prediction, "degrees")\nprint("Absolute prediction:", absolute_prediction, "degrees")')
    md('**Check:** original values predict relative 35° and absolute 15°. They differ because the start positions differ. The resident controller still enforces limits and stop conditions; this arithmetic does not authorize a pose.\n\nFor LIVE only, the instructor chooses a safe joint, enables teleoperation in RViz and demonstrates one small movement while you observe. Students predict direction before the instructor moves. In DEMO discuss the invented trace instead. Do not switch mode while moving.')
    code('report = observe(live=LIVE, duration=3)\nplot_samples(report, JOINT)')
    md('## 4 Explain the twin 3 minutes\n\nCompare one joint angle and its direction with RViz. The twin follows the measured follower stream; a picture is not an independent sensor. A leader–follower angle difference in relative mode can be correct because their start positions differ. Delays, stale data, zero reference errors or an incorrect model can also cause mismatch. A graph alone cannot identify which one.\n\nFind the last fresh teleop status below. Read its reported mode and state. In DEMO these are invented. In LIVE a missing or stale status needs the instructor, not a guess.')
    code('statuses = [s["teleop"]["value"] for s in report["samples"] if s.get("teleop", {}).get("state") == "fresh"]\nprint(statuses[-1] if statuses else "No fresh teleop status. Ask instructor.")')
    md('## 5 Record and rotate 2 minutes\n\nOn your worksheet record: selected joint; predicted relative/absolute angles; one observed direction; DEMO or LIVE; one reason the graph and twin might disagree. Save. Tell the next pair whether your evidence was invented or live.\n\nIf no hardware or ROS is running, keep DEMO and complete exactly the same prediction task. If a LIVE observation fails, ask the instructor; offline data require an explicit switch by restarting without the instructor LIVE setting. Every capture finishes and destroys its ROS node. No background notebook thread remains.')
    for i, cell in enumerate(cells): cell['id'] = f'live-{i}'
    return {'cells':cells,'metadata':{'kernelspec':{'display_name':'Python 3','language':'python','name':'python3'},'language_info':{'name':'python','version':'3.11'}},'nbformat':4,'nbformat_minor':5}


def main():
    path = ROOT / 'notebooks/Robot_Candy_Factory_Live_SO101.ipynb'
    path.write_text(json.dumps(build(), indent=1)+'\n')


if __name__ == '__main__': main()
