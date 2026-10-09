# IR 射击检测算法移植提示词

你正在修改工程 `SHOOT_cmzy_REV081`。请把已经用 Python 逐点回放验证过的双路 ADC 射击检测算法移植到 STM32F072 固件，并接入现有 CAN 应用层。先阅读代码和 `docs/CAN_PROTOCOL.md`，再修改；不要凭旧版 VCNL4040 逻辑推断新传感器行为。

## 已确认的硬件和物理语义

- ADC DMA 原始顺序是 `[ET1, ET2]`。
- 底层物理映射固定为：`ET1/PA1 = barrel_rear`，`ET2/PA3 = barrel_front`。
- 这个映射只能在 ADC/DMA 底层修改。算法层、主循环、CAN 层禁止交换变量名或重新解释 front/rear。
- 当前实测波形是 rear 先出现，front 后出现。
- 两传感器中心距离暂按 `50 mm`，以后可作为一个明确参数修改。
- DAC1/DAC2 约 `1550` 码对应约 `1.25 V`（VDDA=3.3 V），DAC output buffer 已启用。
- ADC 每路实际采样约 `20 kHz`，相邻采样间隔 `50 us`。

## 已验证的第一版算法参数

以下是当前 Python 回放器的默认参数，先原样移植，不要自行增加滞回、冷却时间、多级确认、双向判定或其他未经审核的机制：

```text
front_threshold_offset = +60 ADC
rear_threshold_offset  = +18 ADC
front_max_pulse_width  = 6 ms
rear_max_pulse_width   = 6 ms
rear_min_pulse_width   = 0.3 ms
speed_min              = 2 m/s
speed_max              = 15 m/s
sensor_distance        = 50 mm
```

算法含义：

1. 上电先完成基线采样；校准阶段不生成射击事件。
2. 基线完成后，每个 ADC 原始采样点都按物理语义传给算法层。
3. 事件阈值是各自基线加偏置：`front >= front_baseline + 60`，`rear >= rear_baseline + 18`。
4. 进入阈值时记录该通道的首次进入时间。
5. 在后续采样点中持续追踪该脉冲的 ADC 峰值和峰值时间。
6. 第一个低于阈值的采样点结束脉冲。脉宽超过 `6 ms` 的通道事件丢弃；rear 脉宽短于 `0.3 ms` 的事件丢弃。
7. front 有效脉冲就是一次射击事件，即使 rear 没有有效匹配，也必须计数和上报射击。
8. rear 只作为测速候选，不得否决 front 射击。
9. 配对使用**峰值时间差**，不是进入阈值时间差：

   ```text
   peak_delta_us = front_peak_us - rear_peak_us
   speed_mps = distance_mm * 1000 / peak_delta_us
   ```

   只有 `3.333 ms <= peak_delta_us <= 25 ms`（对应 `2~15 m/s`）才接受测速匹配。rear 必须早于 front。
10. 匹配失败时，射击事件仍然成立；速度字段写入协议约定的无效值，并在调试变量中明确标记 `speed_valid=false`。
11. 一次 front 脉冲只能生成一次射击事件；退出阈值后才允许再次进入。
12. front 峰值前约 2 ms 的下降只作为诊断量记录，例如 `front_pre_dip_adc`，暂时不能作为射击成立的必要条件。

## 基线要求

- 基线必须是算法层的状态，不能由 CAN 层或主循环自行改写。
- 启动校准建议沿用已有 1 s 启动禁检窗口；这段时间只采样基线，不得误触发。
- 初始基线使用启动窗口的中位数或当前工程已实现的等价稳健统计。
- 校准完成后采用当前工程的慢跟踪基线（约 `1/2048` 每采样更新）或与 Python 回放一致的等价实现。
- 当前采样点先做阈值判断，再更新慢基线，避免当前尖峰抬高本点阈值。
- 长时间遮挡、持续高电平和脉宽超过 6 ms 的变化不得生成射击事件。
- 不要把 Ozone 刷新率当作 ADC 采样率。

## 分层和命名要求

必须保持以下层次，禁止跨层转换物理语义：

1. `ir_acquisition`：只负责 DAC、ADC、DMA、采样触发和底层物理映射；向上提供带物理意义的 `barrel_front_adc`、`barrel_rear_adc`。
2. `ir_detection`：只负责基线、阈值、脉冲状态、峰值、峰值时间、rear/front 配对和射击事件；不能直接操作 CAN。
3. 应用层：消费检测层产生的射击事件，更新热量、灯效和状态报告；不能读取 DMA 数组后自行判断或交换 front/rear。
4. `can_protocol`：只负责编码和发送协议报文；不能重新计算 ADC、阈值、峰值或速度。

所有公开变量和函数必须有明确物理语义。禁止使用没有 front/rear/adc/peak/entry/speed 含义的模糊名称。关键映射和时间单位必须写注释：`sample_number` 单位为 50 us，事件时间优先保存为采样序号或 us，避免混用 HAL tick。

## 保留的 SWD 触发原始采样功能

如果链接后的 SRAM 仍有余量，必须保留：

- `ir_raw_capture_dma` 2000 帧双通道环形原始缓冲；
- SWD 写 `ir_raw_capture_arm` 触发一次采集；
- 约 500 帧预触发、1500 帧后触发窗口；
- `ir_raw_capture_done`、`ir_raw_capture_start_word`、`ir_raw_capture_trigger_index`、`ir_raw_capture_valid_frames` 等状态；
- `tool/capture_adc_swd.py` 的连续 `--count` 采集功能；
- 每份采集自动生成 CSV、PNG 和 `.analysis.txt`；
- 细网格图必须保留 50 us 原始点、触发线、峰值坐标和超过 +15/+25/半峰值的脉宽统计。

运行射击检测时，原始捕获功能必须默认不抢占正常 DMA/检测路径。只有 SWD 显式 arm 后才暂停或切换到原始捕获模式；捕获结束后恢复正常采样。若 SRAM 不足，先给出链接 map 中的具体 bss/stack 证据，再只缩小诊断缓冲，不得先删除捕获控制状态或物理映射变量。

## CAN 应用层接入

沿用 `docs/CAN_PROTOCOL.md` 的 500 kbps、标准 11 位 CAN 协议：

- `0x230`：每次 front 有效射击发送一次。
- `Byte0~3`：累计射击次数，小端序。
- `Byte4~5`：速度，编码为 `uint16 = round(m/s × 10)`（分辨率 `0.1 m/s`）；测速无效时使用工程约定的无效值，并保持 `speed_valid` 在内部诊断中可见。
- `Byte6`：射击后的 barrel mask，低 5 位。
- `Byte7`：射击后的热量。
- CAN 发送失败不能撤销射击计数、热量和本地灯效；事件必须留在队列中重试。
- `0x232` 查询回复继续提供累计次数和最近一次速度。
- `0x220` 校准请求仍在主循环执行，不在 CAN 接收中断里做长时间工作。
- 保留现有 `0x200/0x201/0x210/0x211/0x212/0x221/0x233/0x234` 的既有行为，除非代码审查发现明确冲突。

注意：当前旧 `shoot_detect.c` 是 VCNL4040/EXTI 路径，不能与新 `ir_detection.c` 同时对同一发计数。移植时必须明确禁用旧路径的运行调用、旧中断计数和旧队列，或删除其调用边界；不能让两个模块同时发送 `0x230`。

## 必须提供的调试变量

保留并整理为以下可直接在 Ozone 中观察的变量，变量名必须保持物理语义：

```text
barrel_front_adc_raw
barrel_rear_adc_raw
barrel_front_baseline
barrel_rear_baseline
barrel_front_trigger_level
barrel_rear_trigger_level
barrel_front_event_count
barrel_rear_event_count
barrel_front_event_sample
barrel_rear_event_sample
barrel_front_peak_adc
barrel_rear_peak_adc
barrel_front_peak_sample
barrel_rear_peak_sample
ir_shot_count
ir_last_speed_mps
ir_last_speed_valid
ir_last_peak_delta_us
ir_pair_timeout_count
ir_unpaired_front_count
ir_unpaired_rear_count
```

不要继续保留名字含义相反或依赖旧方向的别名。物理方向只能由 `ir_acquisition` 的一处映射决定。

## 验证顺序

不要先改 CAN 再猜检测是否正确。按以下顺序执行：

1. 用 Python 回放器 `tool/simulate_ir_detection.py` 逐点验证 `tool/data`、`tool/data1` 的 12 份数据：front `+60`，rear `+18`，峰值时间差，速度 `2~15 m/s`。
2. 验收当前数据：8 份真实发射各产生 1 次射击，4 份已知误触发产生 0 次；测速应尽量匹配，匹配失败不能影响射击计数。
3. 检查采样缓冲、检测状态和 CAN 事件队列的 SRAM 占用；保留 SWD 捕获功能。
4. 只按 `.vscode/tasks.json` 中已有 task 编译，不要创建 `build_sensor` 或自行发明构建目录；把实际执行的 task 命令和 map/ELF 大小记录下来。
5. 做静态代码审查：确认无旧 `shoot_detect` 双重计数、无 front/rear 交换、无在 CAN ISR 中发送长任务、无用 HAL tick 代替 20 kHz 采样时间。
6. 最后检查 `0x230`：每次 `ir_shot_count` 增加恰好一个事件，CAN 暂时不可用时事件可重试，不能重复计数。

## 工作边界

这次任务的目标是完成“可靠射击计数 + CAN 上报”，测速属于附加结果。不要因为 rear 弱、峰值缺失或速度无效而漏报 front 射击。不要自行增加复杂滤波、双向触发、重新触发门槛、长冷却时间、概率模型或未经数据验证的阈值。任何参数改变都要先写出修改原因、影响和回放结果。
