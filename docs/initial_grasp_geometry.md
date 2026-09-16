# 双孔初态与 USB 抓取几何

> 以下回弯是 `guide.routing: both_guides` 的历史方案。按用户最新选择，默认改为 `usb_to_right`，不再穿左孔；参见 [当前直连布局](usb_direct_routing.md)。USB 抓取位置与 CAD 孔几何的计算仍适用。

本轮正常线缆模式以 `cable_solver:=rope_actor` 为验收后端，MPM 开爪掉落行为留待后续验证。以下双孔几何计算是创建初态检查，不能等同于任一后端的动态抓取结论。

`threaded_centerline()` 现在同时覆盖左、右引导孔。它只生成创建/重置时的中心线；释放临时定位后的线形由真实接触与所选线缆模型演化，不逐帧重写位置。

## 原偏移为什么不能直接沿左孔出线

计算使用当前 `dual_fr3_moveit_config/config/research_franka_hand.xacro` 的装配平移、`research_finger/finger1.STL` 和 `USB1.stl`，全部单位为 m。左右两臂装配相同，手指在局部 TCP 的 X 方向厚度约为 21 mm：`[-10.5, +10.5] mm`；最远指尖位于 TCP `Z=14.4 mm`。孔轴是 TCP X，孔中心是 `TCP + [0,0,2.4] mm`，并非 TCP 原点。沿 `X∈[-12,+12] mm` 的中心线，到闭合 CAD 孔壁最小距离约 `3.5372 mm`。

默认反向安装满足 USB +Y = TCP -X。USB 原始包围盒为 `X=[-3.8,+3.8]`、`Y=[-20,+17.900]`、`Z=[-7.2,+7.2] mm`。三个参考点必须分别计算：

```text
T_tcp_usb = [R_mount, tcp_grip_offset - R_mount * usb.grip_center]
p_grip_tcp = T_tcp_usb * usb.grip_center
p_exit_tcp = T_tcp_usb * usb.attachment
p_hole_tcp = guide.center_offset
```

原抓取偏移 `[0,0,7.5] mm` 产生出线点 `[20,0,7.5] mm`：出线点已在左孔外，USB 自身又占据 TCP `Z=[0.3,14.7] mm`，覆盖孔中心 `Z=2.4 mm`。因此，从 USB 出线直接弯向右手只穿右孔；将线缆强行画穿左孔还会穿入 USB。原偏移下，计算得到左孔中 2 mm 直径线缆与 USB 凸包最大重叠约 **3.10 mm**。不能通过关闭碰撞解决。

当前默认抓取偏移改为 `[0,0,12] mm`，USB 的 TCP Z 范围成为 `[4.8,19.2] mm`。线缆先从 USB 出线，沿 TCP +Z 在指尖外平滑回弯，再从孔另一侧进入左孔；出孔后平滑连接右孔。左孔中 2 mm 线缆表面到 USB 凸包的最小余量为 **1.40 mm**。CAD 指尖仍有夹持重叠区域；真实物理使用的手指凸包会比凹槽 CAD 更早接触，静态顶点检查显示单指位置 `q=4 mm` 已有约 0.074 mm 重叠，而 `q=5 mm` 间隙约 0.924 mm。因此 `usb.finger_position=3.7 mm` 仅保留为旧配置中的名义半开度参考；新接触夹持命令目标为每指 `0 m`，有限驱动力让真实物体阻挡手指，而不是强行达到该位置。抓取是否稳定必须依据释放后的仿真反馈验证。

在相距 0.6 m 的代表性准备位姿上，以 0.5 mm 步长检查完整初态，对两臂各两指原始 CAD（以完全闭合作保守检查；左指夹 USB 时会张开，孔的两半随指运动，报告的孔轴是成对手指的对称参考轴）计算得到线缆表面最小余量 **2.537 mm**。左孔与右孔内径向偏差及轴向角偏差均为数值零；左孔连续直线采样覆盖约 `[-37.80,+37.70] mm`。右孔直线延续到自由端。1.5 m 线长可容纳局部回弯、双孔之间的曲线和自由尾段；不足时创建明确失败。该检查不包含线槽等世界障碍，完整场景仍由后端创建时的碰撞检查把关。 40 mm 回弯在 151 段刚性链重采样后最大弯角约 68.44°，低于 85° 限位；TCP 高度 0.10 m、roll=π 时线缆最低表面约在 z=0.0566 m。线槽各位置高度不同（整体最高约 0.0584 m），不能仅用单一顶面高度判断净空；原准备位姿已通过真实完整场景的创建碰撞检查。更低准备位姿应先检查世界碰撞；不能通过缩小至 30 mm 回弯来规避，此时最大初始弯角约 98.6°，已超限。

## 参数与观测

| 配置 | 默认值 | 作用及依据 |
| --- | ---: | --- |
| `usb.tcp_grip_offset` | `[0,0,0.012] m` | 真实 USB 接触位置，同时让出左孔中的线缆通道 |
| `guide.center_offset` | `[0,0,0.0024] m` | 左右实际孔参考中心，相对各自 TCP |
| `guide.half_length` | `0.012 m` | 两孔必需覆盖范围，比 CAD 半孔长 10.5 mm 多 1.5 mm |
| `guide.initial_margin` | `0.025 m` | 两侧额外直段，实际取不小于两根离散链节/两截面间距及一个线径 |
| `guide.left_loop_clearance` | `0.040 m` | 回弯顶点距孔中心沿 TCP +Z 的距离，位于指尖外；准备姿态须留相应空间 |
| `guide.radial_tolerance` | `0.0001 m` | 初始孔内中心线径向容差，小于线缆半径 |
| `guide.axis_tolerance_deg` | `1 degree` | 初始线缆段相对孔轴允许夹角 |

`initial_layout_report` 分别输出 `left` 和 `right` 的 `radial_error_m`、`axis_error_deg`、`straight_coverage_range_m`、`required_range_m` 和 `passed/reason`。检测使用线段，不要求粗链节节点恰好落在孔中心；遇到外部回弯的多个平面交点时选择径向最近的穿孔交点，再检查连续覆盖。角度使用无向孔轴，正向/反向穿孔等价。报告还输出两孔世界坐标、轴向、USB 出线点与材料弧长，避免混淆 TCP、夹持中心、孔中心和出线点。

`support_material_intervals_m` 只建议临时支持两孔附近的材料小区间。默认每孔约 49 mm，两孔合计约 98 mm，占 1.5 m 线缆不到 7%。场景在夹持准备阶段创建局部世界约束/局部粒子支持，并随 USB 世界定位一起释放；不冻结整根线缆。`external_support=true` 时的直线与稳定不能当成成功抓取。释放后该初态报告仍只记录创建时的几何证据，不能当成当前线缆一直保持直线的声明。

## 复现与边界

工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/dual_fr3_maniskill:$PYTHONPATH" \
  .venv/bin/python src/dual_fr3_maniskill/scripts/check_initial_grasp_geometry.py \
  --output /tmp/initial_grasp_geometry.json

PYTHONPATH="$PWD/src/dual_fr3_maniskill:$PYTHONPATH" \
  .venv/bin/python -m pytest \
  src/dual_fr3_maniskill/test/test_threading.py \
  src/dual_fr3_maniskill/test/test_initial_grasp_geometry.py -q
```

这是 CPU 几何检查，不初始化 ManiSkill、CUDA 或线缆求解器。测试还覆盖不同左右姿态、正反向安装、MPM 截面与稀疏刚性链重采样、材料长度、偏轴/倾斜/覆盖不足的拒绝。配置中的默认线径以当前 YAML 为准（当前两份 trunking 配置均为 2 mm），不能根据历史文档中的 3 mm 实验描述推断。

几何检查通过不等于真实夹持成功；摩擦、动态接触、重力、线缆载荷和释放后的稳定性需另看实际仿真报告。MPM 原有右孔理想滑孔仍属于模型边界；Rope-Actor 释放临时支持后由原始孔 CAD 接触导向，未新增左孔居中约束。
