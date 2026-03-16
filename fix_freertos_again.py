import re

with open("Core/Src/freertos.c", "r", encoding="utf-8") as f:
    text = f.read()

old_loop = """    // --- 一阶低通滤波：解决遥控器信号阶跃导致的电机一卡一卡震动 ---
    smooth_vel_m2 += 0.2f * (target_vel_m2 - smooth_vel_m2);
    smooth_vel_m3 += 0.2f * (target_vel_m3 - smooth_vel_m3);
    smooth_vel_m4 += 0.2f * (target_vel_m4 - smooth_vel_m4);

    // 完全归中时严格清0，防止微小蠕动
    float vel_out_m2 = (fabs(smooth_vel_m2) < 0.01f) ? 0.0f : smooth_vel_m2;
    float vel_out_m3 = (fabs(smooth_vel_m3) < 0.01f) ? 0.0f : smooth_vel_m3;
    float vel_out_m4 = (fabs(smooth_vel_m4) < 0.01f) ? 0.0f : smooth_vel_m4;

    // 更新位置
    pos_ref_m2 += vel_out_m2 * 0.002f;
    pos_ref_m3 += vel_out_m3 * 0.002f;
    pos_ref_m4 += vel_out_m4 * 0.002f;

    // --- 积分补偿计算：防重力缓慢下坠 ---
    float err_m2 = pos_ref_m2 - pos_m2;
    float err_m3 = pos_ref_m3 - pos_m3;

    int_m2 += err_m2 * 0.05f;
    int_m3 += err_m3 * 0.05f;

    // 限制抗重力积分上限（避免积分饱和发疯）
    if(int_m2 > 10.0f) int_m2 = 10.0f;
    if(int_m2 < -10.0f) int_m2 = -10.0f;
    if(int_m3 > 8.0f) int_m3 = 8.0f;
    if(int_m3 < -8.0f) int_m3 = -8.0f;

    float tor_ff_m2 = int_m2;
    float tor_ff_m3 = int_m3;

    // --- 2. 指令下发 ---"""

new_loop = """    // --- 一阶低通滤波：彻底打平 14ms 的遥控器信号阶跃 ---
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
    float tor_ff_m3 = int_m3;

    // --- 2. 指令下发 ---"""

if old_loop in text:
    text = text.replace(old_loop, new_loop)
    with open("Core/Src/freertos.c", "w", encoding="utf-8") as f:
        f.write(text)
    print("Fixed freertos.c filtering and anti-windup.")
else:
    print("Could not find loop to replace in freertos.c")
