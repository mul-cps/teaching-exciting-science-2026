"""MISSION 1 (25 min): Teach the computer to count and make decisions.

Run: python exercises/01_python.py

1. BEFORE running: what will the message say and how many times?
2. Change TEAM to your team's name. Run again.
3. Change REPEATS to 5. Predict the output before running.
4. Change COLOR to 'blue'. What happens? Why?
5. CHALLENGE: Add a separate rule for 'green' without changing the red/blue rules.

Only change the STUDENT AREA first. Everything runs without extra packages.
"""

# ===== STUDENT AREA: change these values =====
TEAM = "Team 1"
REPEATS = 3
COLOR = "red"
# ============================================

print("Welcome to the robot factory,", TEAM)

for number in range(REPEATS):
    print("Object number", number + 1)

if COLOR == "red":
    destination = "left"
elif COLOR == "blue":
    destination = "right"
else:
    destination = "reject"

print("Decision:", destination)
print("Finished. What did you change?")
