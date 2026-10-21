"""Provided notebook drawings. This module never connects to hardware."""
from pathlib import Path

BINS = ("left", "right", "reject")
PAINT = {"red": "#d65854", "blue": "#3889c4", "green": "#238b75", "yellow": "#edbc27", "unknown": "#858b96"}


def draw_bins(routes, colors=None, title="Where did the objects go?"):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    colors = list(colors) if colors is not None else ["unknown"] * len(routes)
    fig, ax = plt.subplots(figsize=(10, 3.6), layout="constrained")
    for x, name in enumerate(BINS):
        ax.add_patch(Rectangle((x * 3.3, 0), 2.8, 2.3, facecolor="#f1f5f9", edgecolor="#203447", linewidth=2))
        chosen = [i for i, route in enumerate(routes) if route == name]
        ax.text(x * 3.3 + 1.4, 2.65, f"{name.upper()}  {len(chosen)}", ha="center", fontsize=13, weight="bold")
        for j, i in enumerate(chosen):
            px, py = x * 3.3 + .5 + (j % 4) * .6, .4 + (j // 4) * .55
            value = colors[i]
            ax.add_patch(Circle((px, py), .22, facecolor=PAINT.get(value, "#858b96") if isinstance(value, str) else "#858b96", edgecolor="#203447"))
            ax.text(px, py, str(i + 1), ha="center", va="center", color="white", fontsize=10)
    rows = max([routes.count(name) for name in BINS] + [0]) // 4
    ax.set(xlim=(-.3, 10), ylim=(-.2, max(3, rows * .55 + 1)), title=title)
    ax.axis("off")
    plt.show()


def show_photos():
    """Real pixel crops with human descriptions, not live classifier output."""
    import matplotlib.pyplot as plt
    from PIL import Image
    folder = Path(__file__).resolve().parents[1] / "notebooks" / "assets"
    fig, axes = plt.subplots(1, 4, figsize=(11, 3.4), layout="constrained")
    labels = [("red", "Human label: red object"), ("blue", "Human label: blue object"),
              ("green", "Human label: green object"), ("uncertain", "No clearly isolated object")]
    for ax, (name, caption) in zip(axes, labels):
        ax.imshow(Image.open(folder / (name + ".jpg")))
        ax.set_title(caption, fontsize=10)
        ax.axis("off")
    fig.suptitle("Crops from the actual teaching station photograph")
    plt.show()


def compare_observations(actual, observed, rules):
    from workshop import Factory, SimBelt, SimCamera, SimRobot
    if len(actual) != len(observed):
        raise ValueError("Use one camera report for each actual object")
    f = Factory(SimBelt(), SimCamera(observed), SimRobot())
    for i, (truth, report) in enumerate(zip(actual, observed), 1):
        route = f.one_cycle(rules)
        print(f"Object {i}: actual={truth!s}, reported={report!s}, destination={route}")
    print("Changed or missing reports:", sum(a != b for a, b in zip(actual, observed)))
    draw_bins(f.robot.actions, actual, "Object colors are actual; routing uses the camera reports")
    return f.counts
