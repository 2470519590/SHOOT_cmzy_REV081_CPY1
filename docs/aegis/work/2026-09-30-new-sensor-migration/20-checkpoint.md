# Checkpoint

> Superseded by the completed ADC-only migration. This historical checkpoint is retained for traceability; its raw-only scope and retained legacy-module statements are no longer runtime requirements.

- Historical completed slice: baseline read and snapshot; dedicated `ir_acquisition.c/.h`; ADC DMA1 Channel1; PA1/PA3 scan; PA4/PA5 DAC; TIM3 20 kHz; USART1; CAN PB8/PB9; old runtime calls disabled.
- Current final slice: `ir_detection` is the sole shot source; main/CAN/thermal/LED integration is complete; VCNL4040, `shoot_detect`, I2C runtime and EXTI sensor routing are retired.
- Evidence: ADC replay across `tool/data` and `tool/data1` produced 8 real-shot events and 4 zero-event captures; final GNU ARM build passed after migration edits; `git diff --check` passed.
- Hardware boundary: no flash or board validation was performed. Ozone, CAN-bus and mechanical-through-barrel validation remain hardware tasks.
- Drift resolution: the historical raw-only scope is superseded by `docs/ir_detection_algorithm_migration_prompt.md` and the current README/CAN/tuning documents.
