# USB 接触夹持验证记录（2026-09-16）

按最终约定，本轮验收使用 **Rope-Actor 和 USB-only**，MPM 动态释放/脱落暂缓。全局默认求解器保持原值；启动时明确指定 `cable_solver:=rope_actor`。

本次验收范围是 USB 端头接触夹持、法向接触载荷采集，以及利用相对位姿真值辅助的稳定性、滑移和脱落监测。没有新增插接基座、插入动作、连续闭环力调节、自动加力或重新抓取。

后续已将 MTC 改为接近前按关键点提前定位，并增加独立偏移状态流；本页保留最初验收记录，最新顺序见 [MTC 使用说明](mtc_cable.md)。

## 可复现检查

在工作区根目录加载 ROS 与工作区后使用项目虚拟环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test src/dual_fr3_moveit_config/test src/dual_fr3_trunking_mtc/test
.venv/bin/python src/dual_fr3_maniskill/scripts/check_initial_grasp_geometry.py \
  --output /tmp/usb_initial_geometry.json
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_grasp_simulation.py \
  --solver rope_actor --output /tmp/usb_grasp_new.jsonl
.venv/bin/python src/dual_fr3_maniskill/test/check_usb_grasp_simulation.py \
  --load-cable --solver rope_actor --output /tmp/usb_grasp_rope_new.jsonl
```

输出文件使用独占创建，换一个尚不存在的路径。实际物理检查需要 Vulkan；MPM 还需要 CUDA。测试加载真实双臂 URDF、USB STL、驱动器和原生接触，脚本中没有给 USB 添加 TCP 固定约束。记录包含每个控制周期的 JSON、观测状态和测试阶段，另生成 `.summary.json`。

`test_usb_only.py` 使用真实 PhysX 场景，但替换了机器人加载器；用于证明后端排除、幂等性及资源生命周期，不能单独证明双臂模型抓稳。`check_usb_grasp_simulation.py` 才是完整模型的接触验证。

## 已完成的物理验证

设备为 NVIDIA RTX 3050 Laptop GPU 4 GiB，SAPIEN 2.2.2 / ManiSkill2 0.5.3。沙箱内 Vulkan 设备创建失败后，在允许访问 GPU 的环境中执行。当前默认线径保留仓库实际配置的 2 mm，默认求解器仍为 MPM；本次推荐命令显式选择 Rope-Actor。

| 检查 | USB-only | Rope-Actor + 线缆 |
| --- | --- | --- |
| 重复创建不增加第二个 USB | 通过 | 通过 |
| 世界临时定位，移动左臂后 USB 世界漂移 | 0 m | 0 m |
| 两指持续接触后才可正常释放 | 通过 | 通过 |
| 释放瞬间位置、线速度和角速度逐项不变 | 通过 | 通过 |
| 无 USB—TCP 固定约束，短暂停留后稳定 | 通过 | 通过 |
| 带物小幅运动无滑移误报 | 通过 | 通过 |
| 开爪后先滑移、后脱落 | 通过 | 通过 |
| 清除与重复清除正确移除物体/支撑 | 通过 | 通过 |
| 不导入/初始化/执行线缆后端，无线缆专用 guide/代理 | 通过 | 不适用 |

稳定观察窗末端，USB-only 相对释放基准的位移为 **0.0214 mm**、转角 **0.0223 rad**；两指法向载荷为 **3.625 / 3.588 N**。Rope-Actor 对应位移 **0.0448 mm**、转角 **0.0209 rad**、两指载荷 **3.658 / 3.663 N**。测试驱动力上限为 10 N/指；这些接触读数不代表电机实际力矩。

Rope-Actor 初始左孔径向偏差 `1.25e-8 m`、轴偏差 `5.91e-5 deg`，直线覆盖约 `[-34.87, +34.81] mm`；右孔径向偏差 `1.45e-8 m`。两孔均完整覆盖要求的 `[-12, +12] mm`。只在创建时检查直线初态；释放后的真实弯曲不靠位置重写消除，`current_guide_alignment` 是只读观测。

局部临时支撑只覆盖两孔附近各约 49 mm 的材料段，约占 1.5 m 线缆的 6.5%。Rope-Actor 使用可移除世界约束；MPM 使用显式临时局部约束及相应 CUDA graph，释放时清除 graph 和支持缓冲区。USB—线缆连接保留。所有这些临时支撑与 USB 世界定位同时释放。

初次使用 60 mm 孔外回弯时，旧准备位姿的线缆与线槽发生 1.723 mm 初始相交，创建被正确拒绝。依据 CAD 和离散弯角计算，将回弯改为 40 mm 后，在原准备位姿通过上述验证；没有关闭碰撞或放宽穿透容差。

最终 Rope-Actor 复验从两侧张开开始，闭合与重新张开的目标速度均为 `0.04 m/s`，七项物理检查全部通过。其记录为 `usb_grasp_rope_final.jsonl` 和 `usb_grasp_rope_final.summary.json`；USB-only ROS 最终记录为 `ros_mtc_usb_only_final.json`。

记录保存在工作区 `artifacts/usb_contact_grasp_20260916/`，索引为 `manifest.json`。原始 JSONL 是运行产物；几何计算依据见 [初始几何说明](initial_grasp_geometry.md)，接口和阈值见 [力与状态输出](forces.md)。

## ROS/MTC 与自动回归

USB-only 实际运行了 MTC 全部五个准备阶段：左夹爪张开、左臂接近、创建、接触闭合、解除定位并验证。日志确认未执行右臂准备或任何布线阶段。检查完成后测试包装器在 100 秒时主动终止 launch；日志末尾的 KeyboardInterrupt/子进程 -2 是清理退出，不是抓持验证失败。随后通过真实 ROS 服务/action 检查：

- `status` 为 `stable`、世界定位已释放；规划场景中已附着 USB。
- 实际世界位姿用于创建规划物体，实际相对位姿用于确认后的规划附着。
- 重复 `spawn`、`release` 不创建物体或改变定位状态。
- 每指 USB 法向载荷可读，线缆通道 disabled/unavailable，基座 unavailable。
- 发送重新张开动作后记录 `dropped`，reset 后记录 `not_created`。

在已经完成 USB-only MTC 抓取的独立终端运行下列检查；它会主动张开并重置仿真物体：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/dual_fr3_maniskill/test/check_usb_grasp_ros.py \
  --output /tmp/usb_ros_acceptance.json
```

力接口专项还使用真实 ROS 订阅比对了 WrenchStamped、JSON 与 JSONL，USB-only 四帧输出通过，缺失基座保持 unavailable。

三包最终自动回归 **468 项通过**（62.89 秒），包括原生 Cartesian 规划、原生 PhysX 后端回归和新增几何/状态/服务生命周期检查；仅有两条上游弃用警告。随后补强非零线速度/角速度的释放保留断言，USB-only 8 项专项再次通过。测试日志保存为 `artifacts/usb_contact_grasp_20260916/final_tests.log`。

三包 `colcon build --symlink-install --packages-select dual_fr3_maniskill dual_fr3_moveit_config dual_fr3_trunking_mtc` 已通过。本机系统 Python 缺少 pytest，ament 提示未注册执行测试；自动回归使用项目 `.venv` 手动执行，不能把构建成功当成测试通过。

## 延期的 MPM 部分

保留 MPM 后端和已接入的生命周期/反馈路径，但不列为本轮接触抓持验收通过。实验中持续接触、撤除支撑及短暂停留能够达到稳定；主动开爪下落触发过轴向连续性或线槽穿透保护。研究过的 MPM 自适应刚体步长没有作为最终实现交付，仍保留原调度和检查阈值。

静态复算提示，MPM USB 出线处的前 7 mm 固定材料段可能进入线槽，而该固定段缺少对应的 PhysX 障碍接触。此项是后续调查线索，不能将调整初始高度或放宽穿透阈值视为修复。已停止扩展该部分。

独立 CUDA 检查仅验证有限局部支撑的生命周期：707 粒子中支撑 70 个，双缓冲各捕获一个 graph；释放前后位置/速度完全不变，graph 重新捕获后不再写支撑位置，粒子渲染实体正确清理。此结果不代表 MPM 整段抓取/脱落通过。

## 验证边界

这些短程测试不代表已完成整条 MTC 布线路径的物理验收，也不代表线缆材料、摩擦或接触传感器已经按实物标定。缺失的 USB 基座通道继续输出 `available=false` 和 `null`。法向接触产生的力矩没有被标为完整摩擦力矩或电机力矩。
