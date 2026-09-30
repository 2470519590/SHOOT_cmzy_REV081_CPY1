# Evidence Bundle

- `make -j4 BUILD_DIR=build_sensor`: PASS; ELF linked at 24636 bytes text, 24 bytes data, 3176 bytes bss; HEX/BIN generated during verification and removed afterward as generated artifacts.
- `uvprojx XML`: PASS.
- `git diff --check`: PASS.
- `arm-none-eabi-nm`: PASS; `adc_dma_buf`, `et1_raw`, `et2_raw`, `adc_half_count`, `adc_full_count`, `adc_error_count`, `ir_dac1_code`, `ir_dac2_code`, `ir_acquisition_ready` present in ELF.
- Static path check: main no longer calls VCNL4040 init/read, ShootDetect calibration/process, or EXTI trigger functions; legacy source modules remain in build/project for future activation.
- Hardware-only boundary: no board flashing, oscilloscope, Ozone runtime, ADC waveform, DAC voltage, CAN PB8/PB9, USART1, or WS2812 electrical verification performed.
- Residual warning: root IOC was rewritten to a coherent new-board snapshot; CubeMX regeneration should be reviewed before use because acquisition timing owner is hand-maintained in `ir_acquisition.c`.
