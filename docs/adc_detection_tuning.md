## ADC 峰谷检测调参（当前版本）

算法逐个 DMA 样本执行：短窗口峰谷差或绝对基线偏差超过阈值，连续确认后产生单路事件。事件期间冻结基线；峰谷差与偏差均低于退出阈值且连续安静后重新布防。后端 ET2 先、前端 ET1 后，FIFO 配对计入 ir_shot_count。此版本提供检测与配对诊断，尚未接入旧 CAN 0x230、热量或射击灯效；gbd_shoot_count 仍是旧检测器数值。

参数在 Ozone Watch 可写，复位恢复默认。

| 参数 | 默认 | 意义 |
|---|---:|---|
| ir_window_samples | 8 | 最近 2～16 个 ADC 样本计算峰谷差，每个样本 50 us；8 点约 0.4 ms 窗口 |
| ir_front_pp_threshold / ir_rear_pp_threshold | 30 | 前端/后端峰谷差触发门限，ADC 码 |
| ir_front_level_threshold / ir_rear_level_threshold | 30 | 前端/后端绝对基线偏差触发门限，ADC 码；补充低速过球，设置 4095 可基本关闭此分支 |
| ir_confirm_samples | 2 | 判定连续成立次数；窗口有重叠，不等于两个独立尖峰 |
| ir_release_percent | 60 | 退出门限为各触发门限乘这个百分比；峰谷差和基线偏差都须低于退出门限 |
| ir_quiet_ms | 80 | 连续安静多久才结束事件，合并同一球多个峰；过大可能合并连续球 |
| ir_rebase_after_ms | 1000 | 事件持续这么久后恢复慢速基线跟踪，适应光照台阶；0 禁用。不能区分卡弹和光照变化；极慢球停留超过此时间也可能被吸收 |
| ir_pair_timeout_ms | 2000 | 后端等前端最长时间，低速测试可调大，最大 60000 |
| ir_pair_min_us | 500 | 两路最小间隔，排除近乎同时扰动；50 mm/20 m/s 是 2500 us |
| ir_distance_mm | 50 | 计算速度使用的传感器间距 |

观察：ir_front_pp_max_5s / ir_rear_pp_max_5s（空管噪声和过球峰谷差）；ir_front_event_count / ir_rear_event_count（单路候选计数）；ir_front_active / ir_rear_active（事件未结束）；ir_shot_count（配对计数）；ir_last_pair_us（两路间隔）；ir_last_speed_mm_s（速度，除以1000得到m/s）；ir_pair_timeout_count / ir_unpaired_front_count / ir_pair_dropped_count（配对失败诊断）。

调参先空管观察，峰谷门限高于空管最大短窗峰谷差；再过一球。单路漏检先调低对应门限，重复触发调大 quiet_ms，空管误触发则提高门限。若只有单路计数增长，查看另一通道、配对方向与超时。慢速测试不能证明20m/s无漏检；高速时不可把确认次数调到接近脉冲样本数，也要按实际球间隔缩短 quiet_ms。
