# Task Intent: New Sensor Migration

- Requested outcome: Implement the first-version DAC + ADC DMA raw acquisition migration in SHOOT_cmzy_REV081 while preserving the existing shot blocking, pairing, speed, thermal, CAN, LED, and IWDG behavior for later sensor-application integration.
- Scope: PA4/PA5 DAC emitters; PA1/PA3 ADC DMA raw samples; stable Ozone symbols; runtime raw-only mode with old VCNL4040/shoot-detect source retained but not active; CAN 500 kbps on PB8/PB9; USART1 interface initialization; USART3 WS2812; build metadata.
- Non-goals: No ADC threshold/direction inference, no new shot events, no false calibration success, no deletion of old algorithm/driver source, no hardware flashing.
- User decision: Preserve algorithms but use raw-only acquisition first; enable application behavior only after Ozone and in-barrel measurements.
- Required baseline refs: docs/SHOOT_cmzy_REV081_new_sensor_migration_plan.md; docs/new_docs/SHOOT_cmzy_REV081.ioc; Core/Src/main.c; Core/Src/stm32f0xx_hal_msp.c; Core/Src/stm32f0xx_it.c; Core/Inc/main.h; Core/Src/can_protocol.c; Core/Src/reliability.c; Makefile; MDK-ARM/SHOOT_cmzy_REV081.uvprojx.
- Baseline state: HEAD fcdc29ac85481cd3a709f2b130f5bc6aa17deb8d on main_xjx, branch ahead 1/behind 3; pre-existing modified/deleted/untracked docs/debug files preserved. Baseline build failed in existing CAN code because can_protocol_tx_permitted() is undefined.
- Change necessity: code-change; new peripheral owner and cross-module wiring are required to implement physical acquisition and debug observability.
- Owner boundary: new IR acquisition module owns ADC/DAC/sampling; main owns scheduling/status publication; MSP owns hardware mappings; IRQ owns DMA callback routing; old shoot_detect/vcnl4040 remain retained but inactive.
- Stop condition: code builds or blockers are explicitly isolated; static checks show old active sensor path is disconnected; real-board/Ozone validation is reported as pending.
