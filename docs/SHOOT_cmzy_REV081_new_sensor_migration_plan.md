# SHOOT_cmzy_REV081 新传感器迁移方案

> 历史方案，已由 `docs/ir_detection_algorithm_migration_prompt.md` 和当前固件实现取代。以下原始采集范围保留用于追溯，不再代表运行时行为。

## 最终状态

旧 VCNL4040 的 I2C/EXTI 采集和 `shoot_detect` 算法已删除。DAC + ADC + DMA 是唯一检测源，并已接入射击计数、可选峰值测速、热量、顶部 LED、CAN `0x230/0x232` 和 `0x220/0x221` 校准流程。

物理映射固定为 `ET1/PA1 -> barrel_rear`、`ET2/PA3 -> barrel_front`。采样间隔 50 us。前端有效脉冲始终产生事件；后端只用于 `3333~25000 us` 峰值时间窗内的可选测速。前/后阈值偏移为 `+60/+18 ADC`，最大脉宽 6 ms，后端最小脉宽 0.3 ms，速度范围 2~15 m/s，距离 50 mm。无效速度编码为 `0xFFFF`，`barrel_mask` 固定为 0。

## 保留的硬件边界

PA4/PA5 DAC、PA1/PA3 ADC DMA、CAN PB8/PB9、USART1、USART3 WS2812 和 IWDG 保持。SWD 原始捕获仍为 2000 帧、500 帧预触发和 1500 帧后触发。未执行刷机；Ozone、CAN 电气和机械过管验证仍需硬件完成。

## 追溯

原 v2.0 文档描述了最初的 raw-only 阶段。该阶段的“保留旧模块”“不产生 `0x230`”“暂不定义算法”结论均已被后续批准迁移和当前代码取代。当前字段和报文以 [CAN_PROTOCOL.md](CAN_PROTOCOL.md) 为准，调参和诊断以 [adc_detection_tuning.md](adc_detection_tuning.md) 为准。
