"""Generated from curriculum/workshop.json. Offline beginner activity.
Run with the prepared Python; no physical commands are sent.
"""
ACTUAL = ["red", "blue", "red"]
REPORTED = ["red", None, "blue"]
for actual, report in zip(ACTUAL, REPORTED):
    print("Actual:", actual, "Reported:", report)
    print("Pause and inspect" if report is None else "Report needs checking")
