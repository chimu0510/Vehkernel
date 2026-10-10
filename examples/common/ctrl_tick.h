/*
 * 定頻控制中斷：TIM6（APB1 計時器時脈 84 MHz，計數 1 MHz）每 1/hz 秒呼叫一次回呼。
 * 控制迴圈（IMU 取樣、PID）放在回呼裡，主迴圈只做顯示與主控台，時序不受它們影響。
 * 中斷優先權 5，高於 SysTick（15）；回呼裡不可呼叫 HAL_Delay()。
 */
#ifndef CTRL_TICK_H
#define CTRL_TICK_H

#include <stdint.h>

typedef void (*ctrl_tick_fn)(void);

/* hz 需整除 1 MHz（例如 100、200、500、1000）。 */
void ctrl_tick_start(uint32_t hz, ctrl_tick_fn fn);
void ctrl_tick_stop(void);

/* 回呼執行時間：最近一次與開始以來的最大值，us。 */
uint32_t ctrl_tick_last_us(void);
uint32_t ctrl_tick_max_us(void);

/* 回呼還沒執行完、下一次中斷就已到達的次數（控制週期太短）。 */
uint32_t ctrl_tick_overruns(void);

#endif /* CTRL_TICK_H */
