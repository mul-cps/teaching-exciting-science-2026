"""Station 1 — Change the text, count and repetition number."""
for number in range(3):
    print("Hello robot", number + 1)

# Start with red or blue. Unknown readings are handled in exercises 03 and 04.
color = "red"
if color == "red":
    print("Move left")
else:
    print("Move right")
