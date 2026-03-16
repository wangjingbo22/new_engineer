import re

with open("Core/Src/freertos.c", "r", encoding="utf-8") as f:
    text = f.read()

old_logic = """    // --- 一阶低通滤波：平滑遥控器输入 ---
    smooth_vel_m2 += 0.05f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.05f * (target_vel_m3 - smooth_vel_m3);
    smooth_vel_m4 += 0.05f * (target_vel_m4 - smooth_vel_m4);

    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;
    float vel_out_m4 = (fabs(smooth_vel_m4) < 0.01f) ? 0.0f : smooth_vel_m4;"""

new_logic = """    // 移除低通滤波，直接响应遥控器
    float vel_out_m2 = target_vel_m2;
    float vel_out_m3 = target_vel_m3;
    float vel_out_m4 = target_vel_m4;"""

if old_logic in text:
    text = text.replace(old_logic, new_logic)
    print("Removed LPF logic.")

# Also handle the initialization and variables
old_vars = """    float smooth_vel_m2 = 0.0f;
    float smooth_vel_m3 = 0.0f;
    float smooth_vel_m4 = 0.0f;

    float int_m2 = 0.0f;
    float int_m3 = 0.0f;"""

new_vars = """    float int_m2 = 0.0f;
    float int_m3 = 0.0f;"""

if old_vars in text:
    text = text.replace(old_vars, new_vars)
    print("Removed LPF variables.")

old_send = """    for(int i = 0; i < num; i++)
    {
      if (motor[i].id == 2) 
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m2, vel_out_m2, kp_m2, kd_m2, tor_ff_m2);
      }
      else if (motor[i].id == 3)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m3, vel_out_m3, kp_m3, kd_m3, tor_ff_m3);
      }
      else if (motor[i].id == 4)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m4, vel_out_m4, kp_m4, kd_m4, 0.0f);
      }
    }"""

new_send = """    for(int i = 0; i < num; i++)
    {
      // [修复]：如果电机不在线(未收到反馈 state==0)，坚决不发CAN数据！
      // 否则空发会导致CAN总线产生ACK应答错误，破坏其他电机的通讯反馈导致抽搐
      if (motor[i].para.state == 0) continue;

      if (motor[i].id == 2) 
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m2, vel_out_m2, kp_m2, kd_m2, tor_ff_m2);
      }
      else if (motor[i].id == 3)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m3, vel_out_m3, kp_m3, kd_m3, tor_ff_m3);
      }
      else if (motor[i].id == 4)
      {
        mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m4, vel_out_m4, kp_m4, kd_m4, 0.0f);
      }
    }"""

if old_send in text:
    text = text.replace(old_send, new_send)
    print("Fixed CAN send logic.")

with open("Core/Src/freertos.c", "w", encoding="utf-8") as f:
    f.write(text)
