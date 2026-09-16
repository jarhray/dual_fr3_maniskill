# USB 接触夹持与可选线缆场景

USB 是独立动态刚体。创建时用可移除的世界约束临时定位，张开夹爪接近后进行有限驱动力闭合；双指持续接触后释放定位，短暂停留验证稳定性。实际 USB 位姿、手指接触载荷及相对 TCP 的滑移/脱落状态沿用现有 TF、WrenchStamped、JSON 和可选 JSONL 输出。接口继续保留 `/usb_cable_demo/*` 命名。

本阶段实现接触夹持、反馈采集和状态判断，正常线缆模式以 `rope_actor` 为本轮验收后端。MPM 保留为后续工作，不宣称其开爪脱落测试通过。没有连续闭环力调节、自动增加夹持力或重新抓取；未加载 USB 基座，没有插入动作或有效基座载荷。

## 启动

按[环境安装与验证](setup.md)构建工作区。工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# USB + Rope-Actor 线缆；本轮验收后端
ros2 launch dual_fr3_moveit_config maniskill.launch.py \
  maniskill_scene:=usb_cable cable_solver:=rope_actor load_cable:=true \
  maniskill_python:="$PWD/.venv/bin/python"

# 仅 USB 接触夹持调试
ros2 launch dual_fr3_moveit_config maniskill.launch.py \
  maniskill_scene:=usb_cable cable_solver:=rope_actor load_cable:=false \
  maniskill_python:="$PWD/.venv/bin/python"
```

分别运行两条启动命令。上面显式选择 `rope_actor`，不会隐式切换用户的后端配置。MPM 与 Rope-Actor 的材料和接触区别见[线缆建模方式](cable_backends.md)。`load_cable:=false` 不创建线缆粒子、刚体链、关节、代理或 guide，不运行线缆 CUDA 内核、Graph 或自适应子步；ManiSkill 自身 GPU/渲染初始化仍可能存在。USB 单独创建不依赖右夹爪穿线；MTC 的 USB-only 模式保留原双臂准备及后续运动轨迹，以测试 USB 搬运偏移，不声称完成实际线缆布线。

`usb_cable.launch.py`、快捷脚本及 `demo.launch.py` 沿用统一启动入口；`cable_config:=...` 控制几何，`maniskill_config:=...` 控制桥接参数。默认线缆场景使用 `simulation_usb_cable.yaml` 的 50 Hz 控制、500 Hz 物理和 25 Hz 发布。不同终端需使用同一 ROS domain。完整 MTC 双孔准备流程见 [MTC 线缆接口](mtc_cable.md)。

## 手动接触夹持流程

独立 `usb_cable` 场景启动时已打开左指，并在初始准备位姿创建世界临时定位的 USB（及可选线缆），可在原地进行闭合验证。若要先用 RViz **Plan & Execute** 改变准备位姿，应先调用 `/usb_cable_demo/reset` 清除现有物体，移动到有足够外部空间的位置后，再按以下顺序重新创建。MTC 的 `trunking_cable` 场景则等待显式 `spawn` 才首次创建。

```bash
# position 是每根手指的关节位置 [m]；总开口约为两倍
ros2 action send_goal /left_franka_gripper/gripper_cmd control_msgs/action/GripperCommand \
  '{command: {position: 0.02, max_effort: 10.0}}'
ros2 service call /maniskill/cable/spawn std_srvs/srv/Trigger '{}'
ros2 action send_goal /left_franka_gripper/gripper_cmd control_msgs/action/GripperCommand \
  '{command: {position: 0.0, max_effort: 10.0}}'
ros2 service call /maniskill/usb/release std_srvs/srv/Trigger '{}'
ros2 service call /maniskill/usb/verify std_srvs/srv/Trigger '{}'
ros2 service call /maniskill/usb/status std_srvs/srv/Trigger '{}'
```

`max_effort` 是驱动力上限 [N]，还受桥接配置/URDF 上限约束，不是测得的实际力。夹爪因接触停住、达到目标开度或 action 成功都不能直接认定抓取成功；`release` 等待双指持续接触，`verify` 只在解除外部支撑后确认稳定。任一步失败先查看原因，不继续假定已抓稳。手动 spawn 未配置目标时使用当时 TCP；MTC 则提前传入关键点准备目标并在接近前创建。两者都只确定一次世界定位，之后不跟随 TCP。

调试时可调用 `/maniskill/usb/release_manual` 强制解除定位，用于故意缺少接触的掉落试验；它不宣称已抓稳。重复创建/释放保持已有对象和状态，不再生成额外 USB。夹持之后仍可用同一 action 将左指打开到 `0.02`，物体允许真实掉落。重置为：

```bash
ros2 service call /usb_cable_demo/reset std_srvs/srv/Trigger '{}'
```

重置清除 USB、线缆及所有临时支持，保留机器人位姿；重新打开手指后再创建。存在活动动作或服务操作时重置会拒绝。释放世界约束时保留位姿和速度，不瞬移、不清零速度、不焊接回 TCP。

## 几何与物理范围

`config/usb_cable.yaml` 保留独立场景的 1.5 m 长、3.5 mm 直径线缆与 `table_spiral` 初态；USB 本体质量为 15 g。现有线密度估计为 15 g/m，1.5 m 线身约 22.5 g，并非已经标定的独立线身质量。MPM 每截面 7 个材料点，默认 1.75 mm 截面间距和 6 mm 网格；轴向纤维约束补充细线刚度。这是实验性混合模型，不是标定的电缆材料模型。

USB STL 按 m 使用、缩放 1，尺寸约 `7.6 × 37.9 × 14.4 mm`；本地 +Y 为插头方向，抓取中心 `(0,0,0)`，出线中心 `(0,-0.02,0)`。动态 USB 在 TCP 中的默认抓取中心偏移是 `(0,0,0.012) m`。双孔 MTC 场景采用独立配置及孔外回弯，实际 CAD 变换、偏移修正证据和双孔直段检查见[双孔初态与 USB 抓取几何](initial_grasp_geometry.md)。

USB 与线缆的物理连接保留。MPM 前 7 mm 材料点固定于动态 USB；Rope-Actor 连接方式由 `root_joint` 决定。这里的 USB—线缆连接与已经移除的 USB—世界定位、规划场景的 attached collision object 是不同概念。

以下 MPM 描述用于说明已有实现，不属于本轮后端验收承诺。MPM 采用体积传递、轴向 XPBD 纤维与有限半径刚体接触。`cable.contact_margin=20 µm`、`penetration_tolerance=0.1 mm`，后者是异常阈值，不是接触厚度。Rope-Actor 使用物理胶囊链和孔 CAD 接触。具体守护与数值限制见[线缆建模方式](cable_backends.md)。原始线缆自接触与摩擦模型不适合据此断言真实打结或精密穿线性能。

柔性线缆不作为 MoveIt 刚性避障物体；规划附着 USB 只能表示规划几何，不能代替物理接触夹持。USB 接触使用 PhysX 碰撞几何，手指凸包和原始孔 CAD 在局部形状上有区别；有限驱动力下实际停留开度可能大于闭合目标，必须看接触与释放后反馈。

## 输出与验证

```bash
ros2 topic echo /maniskill/forces
ros2 topic echo /maniskill/forces/fingers/left_fr3_leftfinger/usb/wrench
ros2 topic echo /maniskill/forces/fingers/left_fr3_rightfinger/usb/wrench
```

力输出规范、状态定义、阈值、JSONL 与验证入口见[力与抓取状态输出](forces.md)。相对位姿 `inverse(T_tcp)*T_usb` 来自仿真真值；机械臂与 USB 整体运动不应算作滑移。接触报告主要是法向冲量，其力矩不是完整摩擦力矩或电机实测力矩。USB-only 的线缆通道为 disabled/unavailable、数值 null；缺失基座通道始终 unavailable。

`/usb_cable_demo/markers` 发布实际线缆中心线，`diagnostics` 继续提供后端的长度、穿透及性能信息；没有线缆时不伪造有效零线缆数据。发生数值失稳时暂停物理、失败原因进入反馈，可通过清理重置恢复。

CAD/状态单元测试不能证明真实抓取成功；实际接触、释放后保持以及开爪脱落必须以真实仿真检查及其 JSONL 报告为依据。本轮验收限于 USB 接触夹持、载荷采集和稳定/滑移/脱落监测，不代表完整 MTC 布线或插接已通过。
