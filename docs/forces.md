# 末端与手指作用力采集

仿真桥接默认启用只读力采集，不改变关节控制器、夹持约束、接触参数或 MTC 动作。
普通机器人、独立 USB 和 MTC 延迟生成线缆场景共用接口。数据用于受力观察和实验记录，
尚未接入力控、抓取判定或插入成功判定。

## 输出形式

| 接口 | 类型 | 内容 |
| --- | --- | --- |
| `/maniskill/forces` | `std_msgs/msg/String`，JSON | 完整时间窗、各传感位置的力/力矩、法向载荷、峰值、接触对象、可用性及缺失分量 |
| `/maniskill/forces/{left,right}/cable/wrench` | `geometry_msgs/msg/WrenchStamped` | 线缆作用于该手及其所持 USB 的载荷，归算到 TCP |
| `/maniskill/forces/{left,right}/fixtures/wrench` | 同上 | 已加载静态结构直接作用于该手及其所持 USB 的载荷 |
| `/maniskill/forces/{left,right}/usb_base/wrench` | 同上 | 上一项中属于指定 USB 插接基座的部分 |
| `/maniskill/forces/fingers/<link>/{cable,fixtures,usb}/wrench` | 同上 | 各手指直接接触相应对象的载荷，归算到手指坐标系原点 |
| `/maniskill/forces/usb/{fixtures,usb_base}/wrench` | 同上 | 静态结构或指定基座直接作用于 USB 的载荷，归算到 USB 坐标系原点 |
| 上述各传感位置下的 `/normal_load` | `std_msgs/msg/Float64` | 可获得时输出法向载荷，单位 N；时间戳和完整含义以 JSON 为准 |

手指 link 为 `left_fr3_leftfinger`、`left_fr3_rightfinger`、
`right_fr3_leftfinger`、`right_fr3_rightfinger`。每个存在的手指单独统计，
两指相反方向的夹持力不会因为腕部合力抵消而丢失。

`WrenchStamped` 仅在该信号可用时发布。消费方必须检查消息时间戳和 JSON 中的
`available`、`scope`、`missing_components`，不能将保留的旧消息视为当前读数。
部分可用的法向接触力仍可画曲线，其 `scope` 为 `partial_interaction_wrench`。

### 坐标、符号和采样

- 力单位 N，力矩单位 N·m，时间为 `/clock` 对应的仿真秒。
- 正负号表示外部对象**作用在传感位置所属刚体上**的力。
- 末端话题使用对应 `*_fr3_hand_tcp` 坐标系；手指和 USB 话题使用自身坐标系。
  力矩包含从接触位置到该坐标系原点的力臂。
- 每个实际物理子步采集一次，控制周期汇总并发布一次。默认线缆场景为 50 Hz，
  不受 `/joint_states` 的 25 Hz 发布频率或原有诊断的墙上时间限频影响。
- 平均值为 `Σ冲量 / Σ实际物理步长`。每个子步先转换到当时的传感坐标系，
  因此是随传感位置转动的局部轴平均值。`frame_poses_world` 记录时间窗末端位姿。
- `peak_substep_force_N` 是窗口内单个物理子步的合力模长峰值，不能解释为连续时间的真实冲击峰值。
- `normal_load_N` 是接触点法向冲量绝对值之和除以时间窗长度，不是合力模长，
  也不是用户设置的夹爪驱动力上限。

## 数据来源与范围

1. **原生刚体接触**：SAPIEN/PhysX 4 报告的法向接触冲量。保留接触对象，
   将 Rope-Actor 手指代理映射回原手指，并且只计一次。
   切向摩擦冲量未由当前报告接口提供，`missing_components` 标记
   `tangential_contact_impulses`；这不意味着物理求解没有摩擦。
2. **Rope-Actor 固定端**：通过 `_rope_physx.read_pair_constraint` 读取线缆首段
   与 USB 之间的约束反力，包含牵引载荷，而非仅统计 USB 表面碰撞。
   PhysX 的约束报告不包含关节驱动项产生的扭矩；柔性根部的弯曲驱动扭矩不能当作已读取。
   缺少扩展、约束断开或约束报告休眠时，对应信号标为不可用。
3. **MPM**：读取现有固定端、刚体接触与理想滑孔代码实际施加的反作用冲量，
   保留力矩，合并当前求解周期内各次耦合反作用。MPM 不额外进行 GPU 接触查询。
   这些是模型施加的反作用载荷；接触汇总未提供逐点法向载荷，因此该标量为 `null`。
4. **MPM 右侧理想滑孔**：约束反力作用在 TCP，不能唯一分配到两根手指。
   右手指通道只含实际接触求解的部分，并标记 `ideal_guide_load_not_resolved_per_finger`。

这些信号按作用来源划分，并非完整腕部六维传感器读数。USB 自重和自身惯性不作为
单独的腕部载荷加入；线缆重力与运动引起的反力会体现在固定端载荷中。
基座接触与线缆载荷分别输出，插入时可以观察两者。

当前左夹爪与 USB 是理想固定夹持，并且排除了安装接触。其 `/usb` 手指通道为
`available=false`、数值 `null`，原因是 `fixed_mount_excludes_usb_finger_contacts`。
不能通过这项修改推导出真实夹紧力；后续改为物理夹持后才能测量。

## 配置和记录

在传给 `maniskill_config` 的 YAML 中设置以下 ROS 参数；其余参数沿用所选场景的配置：

```yaml
dual_fr3_maniskill:
  ros__parameters:
    force_enabled: true
    force_usb_base_names: ""  # 场景中静态 actor 的精确名称；多个名称用逗号分隔
    force_record_path: ""     # 可选，例如 /tmp/forces_run_001.jsonl
```

上述参数在启动时读取，修改后重启桥接。`force_enabled` 默认 `true`。记录路径默认空，避免自动生成大文件；非空时使用独占创建，
已有记录不会被覆盖。记录每个控制周期的 JSON，包含完整元数据、均值、峰值和
`activity`（各臂执行轨迹/保持、夹爪运动/保持）。`activity` 不是 MTC 阶段编号。
每行立即刷新；运行中磁盘写入失败会明确报错并停止文件记录，ROS 发布继续。

`force_usb_base_names` 使用 SAPIEN 场景 actor 名称，不是 STL 路径。
基座必须已经通过场景描述加载为静态结构；采集配置不会自行放置 mesh。
未配置或没有任何匹配的已加载基座时，基座通道标为不可用；
`configured_usb_bases` / `loaded_usb_bases` 可用于核对。
其余已加载结构仍由 `fixtures` 通道统计。无接触且传感通道有效时输出零；
未生成线缆、被排除的夹持接触或缺失基座均使用 `null`。

启动原有 ManiSkill/MTC 流程后可查看：

```bash
ros2 topic echo /maniskill/forces/left/cable/wrench
ros2 topic echo /maniskill/forces/fingers/right_fr3_leftfinger/cable/wrench
ros2 topic echo /maniskill/forces
```

`WrenchStamped.wrench.force.{x,y,z}` 和 `torque.{x,y,z}` 可直接供 ROS 绘图工具使用。
结构化记录也可以通过 rosbag 保存：

```bash
ros2 bag record -o /tmp/force_bag_001 /clock /maniskill/forces \
  /maniskill/left_tcp_pose /maniskill/right_tcp_pose
```

数值故障导致线缆仿真暂停时，结构化话题发布不可用状态，不继续发布旧的力消息。

## 验证

加载 ROS 和工作区后：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  src/dual_fr3_maniskill/test/test_forces.py
```

测试覆盖不等物理步长的平均力与峰值、坐标变换和力矩参考点、接触对顺序、
两指相向受力、代理归属、不可用数据、MPM 空间冲量转换，以及真实 PhysX 中
已知外力/外力矩的约束反力和手指受到已知载荷时的基座接触力。
这些验证不等同于完整双臂走线或真实 USB 插接力学的标定。

实际场景的短时采集、ROS 订阅和 JSONL 写入可用以下脚本验证（需要正常的 Vulkan/CUDA 环境）：

```bash
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 .venv/bin/python \
  src/dual_fr3_maniskill/test/check_forces_simulation.py \
  --solver rope_actor --ros --output /tmp/force_check_rope_001.jsonl
```

`--solver mpm` 验证另一后端。脚本使用记录的准备位姿，检查生成前一次控制周期和
生成后三次控制周期；只验证短时采集链路，不执行完整走线。输出路径必须尚不存在。
