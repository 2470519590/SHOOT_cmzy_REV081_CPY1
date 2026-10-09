# ADC 射击检测调参与诊断

## 模块边界

`ir_acquisition.c/.h` 只负责 DAC、ADC、DMA、采样定时和 ET1/ET2 到物理前后端的唯一映射。`ir_detection.c/.h` 负责基线、阈值、脉冲、峰值、后端候选、前端事件、测速和校准，不访问 DMA 数组或 CAN。`main.c` 负责消费事件、热量、LED、CAN 重试和校准调度。

物理映射固定为：`ET2/PA3 -> barrel_front`，`ET1/PA1 -> barrel_rear`。ADC 样本间隔为 50 us，约 20 kHz/通道。

## 参数

| 参数 | 值 |
|---|---:|
| 前端阈值偏移 | +60 ADC |
| 后端阈值偏移 | +18 ADC |
| 前/后端最大脉宽 | 6 ms |
| 后端最小脉宽 | 0.3 ms |
| 速度范围 | 2~15 m/s |
| 传感器距离 | 50 mm |
| 基线跟踪 | 比较阈值后，约 1/2048 慢速更新 |
| 启动/手动校准 | 约 1 s，稳健中值 |

无额外滞回、连续确认、冷却窗口、多级确认、双向检测或未经验证的滤波。

## 脉冲与峰值

单个样本达到 `baseline + offset` 即进入脉冲；第一个低于进入时阈值的样本退出。退出宽度超过 6 ms 则拒绝。后端有效脉冲进入 8 项峰值候选队列；前端有效脉冲无论是否有后端匹配都生成一个射击事件，且一个脉冲只能生成一次事件，离开阈值后才允许再次进入。

后端必须先于前端。前端事件从时间窗 `3333~25000 us` 内选择最近的、未消费的后端峰值：

```text
peak_delta_us = front_peak_sample - rear_peak_sample
speed_mps = 50 mm / peak_delta
```

若没有匹配或速度超出 `2~15 m/s`，该发仍计入 `ir_shot_count`，但 `ir_last_speed_valid=false`，CAN 速度填 `0xFFFF`。`barrel_mask` 固定为 0。候选队列溢出只影响测速，不影响前端射击数。

## 校准与 CAN

启动校准完成后才生成事件。收到 `0x220` 后，主循环启动新的约 1 s ADC 校准并暂停事件生成；完成后 `0x221` 返回前/后端 baseline 的校准前后四个 `uint16` 小端字段。完整协议见 [CAN_PROTOCOL.md](CAN_PROTOCOL.md)。

## Ozone 诊断变量

- 原始值：`barrel_front_adc_raw`、`barrel_rear_adc_raw`
- 基线/阈值：`barrel_front_baseline`、`barrel_rear_baseline`、`barrel_front_trigger_level`、`barrel_rear_trigger_level`
- 事件入口：`barrel_front_event_count`、`barrel_rear_event_count`、对应 `*_event_sample`
- 峰值：对应 `*_peak_adc`、`*_peak_sample`
- 结果：`ir_shot_count`、`ir_last_speed_mps`、`ir_last_speed_valid`、`ir_last_peak_delta_us`
- 异常：`ir_pair_timeout_count`、`ir_unpaired_front_count`、`ir_unpaired_rear_count`、两路长脉冲计数
- 原始捕获：2000 帧、500 帧预触发、1500 帧后触发，以及 `tool/capture_adc_swd.py` 对应状态变量

原始 DMA 观测变量 `adc_half_count`、`adc_full_count`、`adc_error_count`、DAC 码值和 5 秒峰值窗口仍用于 Ozone/SWD 诊断。它们不改变检测语义。

## 实现边界

校准采用 256 桶直方图选择中值桶，并以桶内低 4 位的 Q8 均值细化基线；这是稳健近似基线，不是完整 12 位精确中位数。恒定输入可以恢复全部 ADC 位，避免旧实现向下偏移最多 15 码。

前端退出时使用已经完成验证的后端候选；同样本退出时先处理后端。尚未退出的后端脉冲不能作为已验证候选，速度可为无效，但前端计数不受影响。

检测事件队列和应用 CAN 重试队列均有 8 项容量。保留在 CAN 队列中的事件重试时不会重复热量或灯效；长时间断连导致队列满后，新报文可能丢失，累计计数、热量和本地灯效继续更新，丢失计数进入弱故障诊断。有限 SRAM 不保证无限断连下每发报文无损；此边界需要系统验收。

## 回放验收

使用 `tool/simulate_ir_detection.py` 对 `tool/data` 与 `tool/data1` 的 12 个 CSV 逐点回放，参数必须为本文件表格中的值。验收目标是 8 个真实过管数据集各 1 发，4 个已知误触发数据集各 0 发；回放通过后再进行固件构建。实板 Ozone、CAN 总线和机械验证不由软件回放替代。
