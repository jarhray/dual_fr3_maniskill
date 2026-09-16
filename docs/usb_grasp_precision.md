# USB 夹持精度与驱动力上限对照（2026-09-16）

本轮按用户要求先完成对照测量，不修改生产默认夹持力、刚度或滑移报警限值；暂停追查原有线缆动态接触问题。

## 结论与实际测量

当前驱动是位置 PD 闭合加有限驱动力上限，`max_effort=10 N` 并不表示每根手指持续施加 10 N。源码 `simulation.py::set_gripper_force` 使用刚度 1000 N/m、阻尼 50 N·s/m。实际驱动力还取决于位置/速度误差，接触载荷由物理解算后独立测量。

使用真实双臂/手指 CAD 和动态 USB，USB-only 模式、不创建线缆。各组依次执行相同准备、有限速度闭合、解除世界定位、静置 0.6 s、小幅机械臂运动 0.5 s、再静置 0.2 s，最后开爪确认脱落。只有试验的固定力上限或固定 PD 刚度不同；阻尼始终 50 N·s/m，USB 摩擦 0.5。没有自适应增力或闭环力调节。

下表统计解除定位后至开爪前约 1.3 s 的数据。位姿基准是解除定位瞬间的 `inverse(T_tcp) * T_usb`，各组均通过真实接触稳定、共同运动不误判及开爪脱落检查。

| 固定力上限/每指 | 固定刚度 | 两指平均法向载荷 | 中心位移峰值 | 转角峰值 | 实际 USB 网格顶点位移峰值 |
| --- | --- | --- | --- | --- | --- |
| 10 N | 1000 N/m | 3.635 / 3.604 N | 0.0312 mm | 1.218° | 0.414 mm |
| 20 N | 1000 N/m | 3.635 / 3.604 N | 0.0312 mm | 1.218° | 0.414 mm |
| 10 N | 2000 N/m | 7.256 / 7.188 N | 0.0498 mm | 1.049° | 0.378 mm |

- 仅将上限翻倍，记录的载荷和位姿序列相同；本工况原 10 N 上限未限制驱动输出。
- 刚度翻倍使法向载荷约翻倍，但中心位移增大，转角只小幅下降。不能从这些数据推出“只需增大夹持力即可达到精密定位”。
- 默认组在静置末尾为约 0.0251 mm / 1.216°，转角在第一个释放后 20 ms 采样点已经约 1.2°，之后保持相近。观测更接近解除支撑后的姿态调整，而不是整个搬运期间持续增大的滑动；具体接触几何和求解误差贡献尚未分离。
- 中心位移不代表 USB 整体定位精度：旋转使真实网格上部分点移动约 0.414 mm。顶点统计由既有相对位姿离线计算，未引入新的物理约束。

这三组为同一模型上的单次受控比较，不是重复性/实物精度标定。法向接触载荷不是完整摩擦载荷；当前证据不能证明所有线缆载荷工况下的夹持力裕量充足。

## 之前接近 0.1 mm 的读数

默认简化线槽的 MTC 记录中，有效 stable 阶段的中心位移峰值约 0.0644 mm，切向 TCP XZ 分量最大约 0.0142 mm，主要中心偏移在夹爪闭合轴 TCP Y。转角约 1.3°。在随后线缆求解数值失效的帧，中心偏移跳到约 0.1073 mm，同时接触载荷也异常跃升；该帧已经标记 failed，不能用它判断真实夹持力不足。

当前 `stable` 表示满足现有保持/脱落判断门限，**不表示达到 0.1 mm 级定位精度**。当前 2 mm / 0.14 rad 的滑移门限用于明显失稳检测。按用户选择先保留这些门限，待确定中心、转角或 USB 关键点的精度目标后再设置精度报警。

## 复现

工作区根目录，使用已经构建的环境和相同默认 simplified 配置：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_grasp_simulation.py \
  --solver rope_actor --grip-force 10 --grip-stiffness 1000 --output /tmp/usb_10N_1000K.jsonl
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_grasp_simulation.py \
  --solver rope_actor --grip-force 20 --grip-stiffness 1000 --output /tmp/usb_20N_1000K.jsonl
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_grasp_simulation.py \
  --solver rope_actor --grip-force 10 --grip-stiffness 2000 --output /tmp/usb_10N_2000K.jsonl
```

不传 `--load-cable` 即为 USB-only；输出文件必须尚不存在。每组自动写 JSONL 和 `.summary.json`，记录实际力上限、刚度与各项检查结果。

原始记录、汇总和曲线位于工作区 `artifacts/usb_precision_20260916/`，其中 `comparison.json` 为表格数值，`comparison.png` 为完整时序。
