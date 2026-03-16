with open("Core/Src/freertos.c", "r", encoding="utf-8") as f:
    text = f.read()

text = text.replace(
    "mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m2, vel_out_m2, kp_m2, kd_m2, tor_ff_m2);",
    "mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m2, vel_out_m2, kp_m2, kd_m2, tor_ff_m2);\n          delay_us(100);"
)

text = text.replace(
    "mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m3, vel_out_m3, kp_m3, kd_m3, tor_ff_m3);",
    "mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m3, vel_out_m3, kp_m3, kd_m3, tor_ff_m3);\n          delay_us(100);"
)

text = text.replace(
    "mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m4, vel_out_m4, kp_m4, kd_m4, 0.0f);",
    "mit_ctrl(&hcan1, &motor[i], motor[i].id, pos_ref_m4, vel_out_m4, kp_m4, kd_m4, 0.0f);\n          delay_us(100);"
)

with open("Core/Src/freertos.c", "w", encoding="utf-8") as f:
    f.write(text)

