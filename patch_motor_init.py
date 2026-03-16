import re

with open("User/dm_motor_ctrl.c", "rb") as f:
    data = f.read()

pattern = b'(motor\\[Motor3\\]\\.ctrl\\.tor_set\\s*=\\s*0\\.0f;[^\n]*\n)'

replacement = b'\\1\tmotor[Motor3].tmp.PMAX\t\t= 12.5f;\n\tmotor[Motor3].tmp.VMAX\t\t= 45.0f;\n\tmotor[Motor3].tmp.TMAX\t\t= 18.0f;\n'

new_data, count = re.subn(pattern, replacement, data)
if count > 0:
    with open("User/dm_motor_ctrl.c", "wb") as f:
        f.write(new_data)
    print("Patched Motor 3 initialization!")
else:
    print("Failed to find pattern")
