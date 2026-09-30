# Checkpoint

- Completed: baseline read and snapshot; added dedicated `Core/Src/ir_acquisition.c` and header; enabled ADC HAL and ADC DMA1 Channel1; configured PA1/PA3 scan with circular DMA; configured PA4/PA5 DAC output buffers and adjustable codes; configured TIM3 TRGO at 20 kHz; configured USART1 PB6/PB7; moved CAN mapping to PB8/PB9; disabled old EXTI/I2C runtime calls; retained shoot_detect/vcnl4040 sources and CAN/thermal/LED/IWDG code; fixed pre-existing undefined heartbeat helper reference; GCC build passed.
- Active slice: final integration review and verification.
- Evidence: `make -j4 BUILD_DIR=build_sensor` passed, ELF size 24644 text / 3176 bss; `git diff --check` passed before latest IOC edits.
- Remaining: re-run build after final edits; inspect old-path references and project metadata; document Ozone variables and hardware-only validation; clean generated build dirs from task delta if appropriate without touching pre-existing user files.
- Drift: scope remains raw-only first version; legacy application algorithms retained but inactive; no persistent state or external contract deletion.
