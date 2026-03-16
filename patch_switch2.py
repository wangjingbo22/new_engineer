import re

with open("Core/Src/freertos.c", "rb") as f:
    data = f.read()

pattern = b'/\\* Infinite loop \\*/[ \r\n]*for\\(;;\\)[ \r\n]*{'

replacement = b'uint8_t last_s0 = 0;\n    uint8_t last_s1 = 0;\n    /* Infinite loop */\n    for(;;)\n    {'

new_data, count = re.subn(pattern, replacement, data)
if count > 0:
    with open("Core/Src/freertos.c", "wb") as f:
        f.write(new_data)
    print("Injected state variables")
else:
    print("Failed to inject")
