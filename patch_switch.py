import re

with open("Core/Src/freertos.c", "rb") as f:
    data = f.read().decode('utf-8', errors='ignore')

# To implement smooth transition when switching gears, we need to track previous states.
# If previous state was not 1,1 and now is 1,1, we should sync pos_ref to current position.
# Let's check if the previous state variables exist.
