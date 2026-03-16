import re

with open("User/dm_motor_drv.c", "r", encoding="utf-8") as f:
    text = f.read()

old_func = """int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
\t/* Converts a float to an unsigned int, given range and number of bits */
\tfloat span = x_max - x_min;
\tfloat offset = x_min;
\treturn (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}"""

new_func = """int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
\t/* Converts a float to an unsigned int, given range and number of bits */
\tif (x_float < x_min) x_float = x_min;
\tif (x_float > x_max) x_float = x_max;
\tfloat span = x_max - x_min;
\tfloat offset = x_min;
\treturn (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}"""

if old_func in text:
    text = text.replace(old_func, new_func)
    with open("User/dm_motor_drv.c", "w", encoding="utf-8") as f:
        f.write(text)
    print("Fixed dm_motor_drv.c float_to_uint clamp.")
else:
    print("Could not find float_to_uint in dm_motor_drv.c")
