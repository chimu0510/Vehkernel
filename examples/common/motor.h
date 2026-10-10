/*
 * 馬達 A、B 經接口 P2 驅動 D50A（腳位見 board.h），PWM 20 kHz。
 * duty > 0：IN1 = H、IN2 = L；duty < 0：IN1 = L、IN2 = H；duty = 0：兩腳皆 L（脫機）。
 * IN1/IN2 接到 D50A 的 INA/INB 哪一腳由排線決定，實際轉向以 32_motor 量測為準。
 */
#ifndef MOTOR_H
#define MOTOR_H

typedef enum {
    MOTOR_A = 0,
    MOTOR_B,
    MOTOR_COUNT,
} motor_id_t;

void motor_init(void);

/* duty 範圍 -1.0 .. 1.0，超出範圍會被截斷。 */
void motor_set(motor_id_t id, float duty);

/* 兩路立即脫機並把 PWM 歸零。 */
void motor_stop_all(void);

#endif /* MOTOR_H */
