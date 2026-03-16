import re

with open("Core/Src/freertos.c", "r", encoding="utf-8") as f:
    data = f.read()

pattern = r'(if \(rc\.s\[0\] == 1 && rc\.s\[1\] == 1\)\s*\{\s*// \[原逻辑\] s\[0\]=1 且 s\[1\]=1：控制大小臂)'
replacement = r'''if (rc.s[0] == 1 && rc.s[1] == 1)
      {
        // 档位切换时平滑接管
        if (!(last_s0 == 1 && last_s1 == 1)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 2) pos_ref_m2 = motor[i].para.pos;
                if (motor[i].id == 3) pos_ref_m3 = motor[i].para.pos;
            }
        }
        // [原逻辑] s[0]=1 且 s[1]=1：控制大小臂'''
new_data, count = re.subn(pattern, replacement, data)
if count > 0:
    data = new_data
    print("Injected state sync for arm")
    
pattern = r'(else if \(rc\.s\[0\] == 2 && rc\.s\[1\] == 2\)\s*\{\s*// \[新逻辑\] s\[0\]=2 且 s\[1\]=2：控制末端夹爪)'
replacement = r'''else if (rc.s[0] == 2 && rc.s[1] == 2)
      {
        // 档位切换时平滑接管
        if (!(last_s0 == 2 && last_s1 == 2)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 4) pos_ref_m4 = motor[i].para.pos;
            }
        }
        // [新逻辑] s[0]=2 且 s[1]=2：控制末端夹爪'''
new_data, count = re.subn(pattern, replacement, data)
if count > 0:
    data = new_data
    print("Injected state sync for claw")

pattern = r'(osDelay\(2\);\s*// 500Hz control loop\s*\})'
replacement = r'''last_s0 = rc.s[0];
      last_s1 = rc.s[1];
      osDelay(2); // 500Hz control loop
    }'''
new_data, count = re.subn(pattern, replacement, data)
if count > 0:
    data = new_data
    print("Injected state variable tracking")

with open("Core/Src/freertos.c", "w", encoding="utf-8") as f:
    f.write(data)
