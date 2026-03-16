with open("User/bsp_can.c", "r") as f:
    lines = f.readlines()

with open("User/bsp_can.c", "w") as f:
    for line in lines:
        if "uint32_t wait_timeout = 0xFFFF" in line:
            f.write(" EOF
^C
 `echo "\004"`
 cat -n /home/abc/Desktop/engineer/3_1/User/bsp_can.c | sed -n '61,70p' > tmp_check.txt
cat tmp_check.txt
 
