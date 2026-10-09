# SHOOT_cmzy_REV081 枪管固件

## 项目职责

固件运行于 STM32F072C8x，负责双路红外 ADC 检测、射击计数与测速、热量和 LED 状态，以及 500 kbps 标准 CAN 通信。检测链路已经完全迁移到 DAC + ADC + DMA；运行时不再使用 VCNL4040、I2C 或旧 `shoot_detect` 检测器。

## ADC 采集与物理命名

- PA4/PA5：DAC 红外发射端，默认码值由 `ir_acquisition.h` 定义。
- PA1/ET1：物理后端，统一命名为 `barrel_rear`。
- PA3/ET2：物理前端，统一命名为 `barrel_front`。
- ADC DMA 原始顺序为 `[ET1, ET2, ...]`，50 μs 一个样本对，约 20 kHz/通道。
- `ir_acquisition` 只负责 DAC、ADC、DMA、定时和物理映射；`ir_detection` 只负责算法和诊断；`main` 负责业务副作用与调度。
- SWD 原始捕获保留 2000 帧结构：500 帧预触发、1500 帧后触发。

## 检测算法

启动和 `0x220` 重校准使用约 1 s 采样窗口，并以稳健中值建立两路基线。运行时先比较样本与阈值，再按约 `1/2048` 慢速更新基线。

```text
front_threshold = front_baseline + 60 ADC
rear_threshold  = rear_baseline + 18 ADC
front/rear 最大脉宽 = 6 ms
rear 最小脉宽 = 0.3 ms
速度范围 = 2~15 m/s
传感器间距 = 50 mm
```

物理顺序为后端先、前端后。前端每个有效脉冲始终产生一个且仅一个射击事件；后端只作为可选测速候选。测速使用峰值时间差：

```text
peak_delta_us = front_peak_us - rear_peak_us
3.333 ms <= peak_delta_us <= 25 ms
speed = 50 mm / peak_delta
```

后端候选使用 8 项队列，前端事件选择时间窗内最近的、尚未消费的后端峰值。没有有效匹配时速度无效，但前端射击计数不受影响。`barrel_mask` 固定为 `0`。

主要 Ozone 变量包括：`barrel_front_adc_raw`、`barrel_rear_adc_raw`、两路 baseline/trigger level、事件数和采样号、峰值 ADC/采样号、`ir_shot_count`、`ir_last_speed_mps`、`ir_last_speed_valid`、`ir_last_peak_delta_us`、`ir_pair_timeout_count`、`ir_unpaired_front_count` 和 `ir_unpaired_rear_count`。

## 应用链路

`main` 从检测层取出事件后执行一次本地计数关联、每发热量 `+10`（上限 200）和顶部 LED 效果，然后将同一个事件交给 CAN 发送队列。CAN 发送失败只保留队列事件重试，不重复执行本地热量和灯效。

- `0x230`：累计次数、速度（`uint16 = round(m/s × 10)`（分辨率 0.1 m/s），无效为 `0xFFFF`）、固定 `barrel_mask=0`、本发后热量。
- `0x232`：累计次数、最近速度（无效为 `0xFFFF`）、固定 `barrel_mask=0`、当前热量。
- `0x220`：主循环启动约 1 s ADC 重校准，校准期间不生成事件。
- `0x221`：前/后端校准前后 baseline，各一个小端 `uint16`。

完整字段和接收约束见 [docs/CAN_PROTOCOL.md](docs/CAN_PROTOCOL.md)。

## 目录

| 路径 | 用途 |
|---|---|
| `Core/Src/ir_acquisition.c` | DAC、ADC、DMA、采样定时和物理通道映射 |
| `Core/Src/ir_detection.c` | 基线、阈值、脉冲、峰值、候选匹配、事件和校准 |
| `Core/Src/main.c` | 初始化、事件消费、热量、LED、CAN 重试和任务调度 |
| `Core/Src/can_protocol.c` | CAN 编码、发送和接收协议 |
| `Core/Src/thermal.c` | 热量累加与冷却 |
| `Core/Src/led_rgb.c` | 顶部灯、队伍色、热量条和故障显示 |
| `Core/Src/reliability.c` | 弱故障及复位记录 |
| `tool/simulate_ir_detection.py` | ADC CSV 逐点回放参考实现 |
| `docs/adc_detection_tuning.md` | ADC 检测参数与诊断说明 |

## 构建与验证

使用现有 VS Code task `STM32: Build`，等价于：

```text
E:\GnuWin32\bin\make.exe -j2 all
```

软件验证包括 Python 数据回放、旧路径静态搜索、`git diff --check`、固件构建以及 ELF 的 SRAM/Flash 使用检查。未执行刷机；实板 Ozone 观察、CAN 总线和机械过管验证仍需在硬件环境完成。
