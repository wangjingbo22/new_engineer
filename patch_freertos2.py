import re

with open("Core/Src/freertos.c", "r", encoding="utf-8") as f:
    text = f.read()

old_params = """  float kp_m2 = 450.0f;    // 提高刚度，配合积分项强力锁死
  float kd_m2 = 5.0f;      
  
  float kp_m3 = 350.0f;    // 提高小臂刚度
  float kd_m3 = 4.0f;      

  float kp_m4 = 100.0f;     // 末端夹爪 4310
  float kd_m4 = 3.0f;"""

new_params = """  float kp_m2 = 180.0f;    // 降低大臂刚度，消除机械卡顿与震颤
  float kd_m2 = 4.0f;      // 保持合理的阻尼
  
  float kp_m3 = 120.0f;    // 降低小臂刚度
  float kd_m3 = 3.0f;      

  float kp_m4 = 40.0f;     // 夹爪不需要太强，防止堵转抢电拖垮母线导致掉臂
  float kd_m4 = 1.0f;"""

if old_params in text:
    text = text.replace(old_params, new_params)
    print("Replaced params")

old_logic = """    if (rc.s[0] == 1 && rc.s[1] == 1)
      {
        // 档位切换时平滑接管，同时清零重力积分补偿
        if (!(last_s0 == 1 && last_s1 == 1)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 2) { pos_ref_m2 = motor[i].para.pos; int_m2 = 0.0f; }
                if (motor[i].id == 3) { pos_ref_m3 = motor[i].para.pos; int_m3 = 0.0f; }
            }
        }
        
      float ch3_val = (float)rc.ch[3];
      float ch1_val = (float)rc.ch[1];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;
      if (fabs(ch1_val) < 20.0f) ch1_val = 0.0f;

      target_vel_m2 = ch3_val * 0.02f; 
      target_vel_m3 = ch1_val * 0.02f; 
    }
    else if (rc.s[0] == 2 && rc.s[1] == 2)
      {
        if (!(last_s0 == 2 && last_s1 == 2)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 4) pos_ref_m4 = motor[i].para.pos;
            }
        }
        
      float ch3_val = (float)rc.ch[3];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;

      target_vel_m4 = ch3_val * 0.05f; 
    }

    // --- 一阶低通滤波：彻底打平 14ms 的遥控器信号阶跃 ---
    // 0.02f 的系数能在 100ms 内平滑完成响应，让运动彻底告别卡顿
    smooth_vel_m2 += 0.02f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.02f * (target_vel_m3 - smooth_vel_m3);
    smooth_vel_m4 += 0.02f * (target_vel_m4 - smooth_vel_m4);

    // 完全归中时严格清0，防止微小蠕动
    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;
    float vel_out_m4 = (fabs(smooth_vel_m4) < 0.01f) ? 0.0f : smooth_vel_m4;

    // 更新位置 (每 2ms = 0.002s 执行一次)
    pos_ref_m2 += vel_out_m2 * 0.002f;
    pos_ref_m3 += vel_out_m3 * 0.002f;
    pos_ref_m4 += vel_out_m4 * 0.002f;

    // --- 锁止积分补偿：仅在停止期间进行防重力抗下坠 ---
    float err_m2 = pos_ref_m2 - pos_m2;
    float err_m3 = pos_ref_m3 - pos_m3;

    // 为了防止移动期间误差过大导致积分器风饱和，仅当速度为 0 时引入重力补偿
    if (fabs(vel_out_m2) < 0.02f) {
        int_m2 += err_m2 * 0.08f;
    }
    if (fabs(vel_out_m3) < 0.02f) {
        int_m3 += err_m3 * 0.08f;
    }

    // 限制抗重力积分上限
    if(int_m2 > 12.0f) int_m2 = 12.0f;
    if(int_m2 < -12.0f) int_m2 = -12.0f;
    if(int_m3 > 10.0f) int_m3 = 10.0f;
    if(int_m3 < -10.0f) int_m3 = -10.0f;

    float tor_ff_m2 = int_m2;
    float tor_ff_m3 = int_m3;"""

new_logic = """    if (rc.s[0] == 1 && rc.s[1] == 1)
      {
        // 档位切换时平滑接管，不轻易清零力矩，防止掉落
        if (!(last_s0 == 1 && last_s1 == 1)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 2) pos_ref_m2 = motor[i].para.pos;
                if (motor[i].id == 3) pos_ref_m3 = motor[i].para.pos;
            }
        }
        
      float ch3_val = (float)rc.ch[3];
      float ch1_val = (float)rc.ch[1];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;
      if (fabs(ch1_val) < 20.0f) ch1_val = 0.0f;

      target_vel_m2 = ch3_val * 0.02f; 
      target_vel_m3 = ch1_val * 0.02f; 
    }
    else if (rc.s[0] == 2 && rc.s[1] == 2)
      {
        if (!(last_s0 == 2 && last_s1 == 2)) {
            for(int i = 0; i < num; i++) {
                if (motor[i].id == 4) pos_ref_m4 = motor[i].para.pos;
            }
        }
        
      float ch3_val = (float)rc.ch[3];
      if (fabs(ch3_val) < 20.0f) ch3_val = 0.0f;

      target_vel_m4 = ch3_val * 0.015f;  // 降低夹爪移动速度，防堵转
    }

    // --- 一阶低通滤波：平滑遥控器输入 ---
    smooth_vel_m2 += 0.05f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.05f * (target_vel_m3 - smooth_vel_m3);
    smooth_vel_m4 += 0.05f * (target_vel_m4 - smooth_vel_m4);

    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;
    float vel_out_m4 = (fabs(smooth_vel_m4) < 0.01f) ? 0.0f : smooth_vel_m4;

    // 更新目标位置 
    pos_ref_m2 += vel_out_m2 * 0.002f;
    pos_ref_m3 += vel_out_m3 * 0.002f;
    pos_ref_m4 += vel_out_m4 * 0.002f;

    // --- 弹簧风饱和与跟随截断 (Elastic Wind-up Limit) ---
    // 强制限制目标位置不能超前当前物理位置太多！
    // 彻底解决小臂松开摇杆时依然“追赶而多走一段（超前不受控）”的问题
    float err_m2 = pos_ref_m2 - pos_m2;
    if (err_m2 > 0.12f) { pos_ref_m2 = pos_m2 + 0.12f; err_m2 = 0.12f; }
    if (err_m2 < -0.12f) { pos_ref_m2 = pos_m2 - 0.12f; err_m2 = -0.12f; }

    float err_m3 = pos_ref_m3 - pos_m3;
    if (err_m3 > 0.12f) { pos_ref_m3 = pos_m3 + 0.12f; err_m3 = 0.12f; }
    if (err_m3 < -0.12f) { pos_ref_m3 = pos_m3 - 0.12f; err_m3 = -0.12f; }

    float err_m4 = pos_ref_m4 - pos_m4;
    // 夹爪限制更死，限制最大堵转力量，绝对不会拖垮母线导致大臂掉电
    if (err_m4 > 0.15f) { pos_ref_m4 = pos_m4 + 0.15f; err_m4 = 0.15f; }
    if (err_m4 < -0.15f) { pos_ref_m4 = pos_m4 - 0.15f; err_m4 = -0.15f; }

    // --- 极柔性的静差抗拽重力补偿 ---
    // 即使在运动也会保持极低速率的恒定重力累加，由于 err 被上一段截断，它绝对不会发疯
    // 这个极其微弱的值 (0.005) 不会因为夹爪传来的机械震颤而产生暴力的错误扭矩
    int_m2 += err_m2 * 0.005f;
    int_m3 += err_m3 * 0.005f;

    // 限制重力补偿上限
    if(int_m2 > 12.0f) int_m2 = 12.0f;
    if(int_m2 < -12.0f) int_m2 = -12.0f;
    if(int_m3 > 8.0f) int_m3 = 8.0f;
    if(int_m3 < -8.0f) int_m3 = -8.0f;

    float tor_ff_m2 = int_m2;
    float tor_ff_m3 = int_m3;"""

if old_logic in text:
    text = text.replace(old_logic, new_logic)
    print("Replaced logic")

with open("Core/Src/freertos.c", "w", encoding="utf-8") as f:
    f.write(text)

