with open("User/bsp_can.c", "r", encoding="gbk") as f:
    lines = f.readlines()

with open("User/bsp_can.c", "w", encoding="gbk") as f:
    for i, line in enumerate(lines):
        if "uint32_t wait_timeout = 0xFFFF;" in line:
            f.write(" echo -e '\x03' 
 python3 -c '
with open("User/bsp_can.c", "r", encoding="gbk", errors="ignore") as f:
    lines = f.readlines()
with open("User/bsp_can.c", "w", encoding="gbk") as f:
    for i, line in enumerate(lines):
        if "uint32_t wait_timeout = 0xFFFF;" in line:
            f.write("uint32_t wait_timeout = 0x0FFFFF;\n")
        elif "HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);" in line:
            f.write("if (wait_timeout > 0) {\n")
            f.write("HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);\n")
            f.write("return 1;\n")
            f.write("} else { return 0; }\n")
        elif "return 0;" in line and "HAL_CAN_AddTxMessage" in lines[i-1]:
            pass
        else:
            f.write(line)
'
cat -n User/bsp_can.c | sed -n '60,70p'
 EOF
 cat User/bsp_can.c | sed -n '61,70p' > /tmp/target.txt
xxd /tmp/target.txt
 cat -n Core/Src/freertos.c | sed -n '288,308p'
 
