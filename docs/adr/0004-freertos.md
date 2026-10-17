# RTOS 採用 FreeRTOS

選擇 FreeRTOS 而不是 Zephyr，因為它最穩定、資源最多，在 STM32F4 上也最成熟。這個選擇的代價是多板支援得靠自己：Zephyr 的 devicetree 能直接解決板級定義，FreeRTOS 沒有，所以韌體要自建板級定義與 HAL 抽象層（參考 ArduPilot 的 hwdef 與 AP_HAL），載具邏輯不可直接呼叫 RTOS 或廠商 HAL。
