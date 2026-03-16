import re

with open("Core/Src/freertos.c", "rb") as f:
    data = f.read().decode('utf-8', errors='ignore')

print("Checking else block")
