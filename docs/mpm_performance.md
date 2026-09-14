# MPM 性能与验证

本页记录截至 2026-09-14 整理的 MPM 实现，历史性能数据测于 2026-09-12。
当前仅实现 MPM 后端；CUDA Graph 与 GPU 网格检查默认开启，轴向迭代保留 6。
安装步骤见 [环境安装与验证](setup.md)，所有命令从工作区根目录执行。

## MTC 线缆性能基准

停止其他仿真后，在已加载 ROS 和工作区、已安装两个仿真包的环境运行：

```bash
.venv/bin/python src/dual_fr3_maniskill/scripts/profile_trunking_cable.py \
  --steps 3 --warmup-steps 1 --render-frames 3 \
  --output /tmp/trunking-profile
```

输出目录必须尚不存在，以免覆盖旧结果。脚本使用当前 `trunking_cable.yaml` 和
`simulation_usb_cable.yaml`，保留物理参数，在固定准备姿态下保持机械臂目标不动。
默认姿态见 `config/profiling_trunking_pose.json`，来自一次 MTC 准备 IK 输出，数值保留四位小数。
可用 `--initial-positions` 指定包含左右臂共 14 个关节名及弧度值的 JSON；夹爪宽度取自场景配置。
该脚本直接创建独立仿真，不创建 ROS 动作客户端，不执行完整 MTC，也不连接真实机器人。

基准分为原始执行和加入计时的两轮。两轮从相同姿态创建场景、执行相同预热与测量步数；
预热、资源加载和 CUDA 首次编译不计入步进耗时。`report.json` 保存参数、输入哈希、版本、
每步耗时、分段/内核耗时、CPU/GPU 拷贝次数与字节数、数值状态差异；`report.md` 为摘要，
`*_state.npz` 为末态。输出原始执行结果后才开始分段计时，第二轮失败时仍可查看 `baseline.json`。

分段计时会在 Warp 内核和拷贝完成后同步，避免将异步内核的等待时间误记到后续 `.numpy()`。
这些耗时包含主机提交、驱动及等待，并非纯 CUDA 设备事件时间；额外同步会扰动执行，
应结合两轮总耗时比评估。组件表已扣除嵌套组件，内核/拷贝表是其中的明细，不能再相加。
渲染单独测量离屏 RGB 图像及像素回读，不代表交互窗口或 RViz 的开销。
短时静止姿态结果不能直接外推到整个运动过程，或替代长期稳定性验证。

## CUDA Graph 与 GPU 网格检查

`mpm.cuda_graph: true` 将每个 MPM 子步中的轴向纤维、右手导向、碰撞投影序列录制为 CUDA Graph。
它减少主机逐内核提交的开销，保留原有内核顺序、迭代数和刚体耦合频率。两个粒子状态分别缓存 Graph；
夹持目标、右手姿态、刚体状态和纤维反作用力所用质心通过持续存在的 GPU 缓冲区更新，不能冻结为首帧参数。
改变时间步、迭代数、相关标量配置、网格分配或重置线缆会重新录制，关闭场景时释放 Graph。
缺少 Graph API、启用 Warp `verify_cuda` 或 Graph 实例化失败时，会提示并使用普通 CUDA 提交。
内核编译失败、数值异常仍直接报错，不视为可忽略的性能问题。

`mpm.gpu_grid_check: true` 在 GPU 上计算全部粒子的位置极值和有效性，常规子步只回读 32 字节摘要。
接近分配边界时采用保守判断，并把六个极值交给原 `grid_layout()` 规则复核；需要扩容时仍执行原有内存预算和
自适应分辨率逻辑。实际 MPM 网格原点计算保持原样，每个子步的数值错误和接触缓冲区检查也继续保留。
这项改变减少全量粒子回读及 CPU 扫描，不表示已经消除了逐子步同步。
旧配置文件省略这两个字段时均默认启用；排查另一台机器或做消融对照时，可分别设置 YAML 布尔值 `false`。

性能脚本会把 Graph 重放列为 `graph/constraints`，计入 `fiber/guide/contact sequence`。
录制过程不计为已执行的内核，也不能在录制中插入同步。要观察 Graph 内部的逐内核耗时，可用
`mpm.cuda_graph: false` 的配置再次运行；`--contact-details` 的独立计数轮也会禁用约束 Graph 重放，
因此不能用该轮的耗时计算 Graph 加速比。

GPU 边界、相同输入 Graph 对照、移动姿态、双缓冲、扩容、重置、回退与销毁检查：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_execution_cuda.py \
  --output /tmp/trunking-execution-check
.venv/bin/python src/dual_fr3_maniskill/test/check_execution_motion_cuda.py \
  --steps 20 --position-limit-mm 0.16 --output /tmp/trunking-execution-motion
```

第二项比较普通执行两轮、加速后的 6 次迭代和 4 次迭代，每轮含运动 20 步、重置后 5 步，并保存每个控制步的粒子位置。
0.16 mm 是明确指定的位置坐标差预算，不改变 0.1 mm 穿透限制；RMS、速度、附件约束也各自检查。
所有原始结果（包括不通过项）保留在 JSON 中，严格位置检查失败会非零退出。

本机 2026-09-12 对照中，4 次迭代的运动末态差约 16.3 mm，因此 **`axial_iterations` 保留 6**。
加速后的 6 次迭代末态差约 0.100 mm；新增的全程 0.16 mm 检查未通过，过程峰值约 0.175 mm，
而普通执行自身重复的过程峰值约 0.233 mm。相同输入的 40 组约束链对照中，位置、速度、APIC 状态一致，
仅累计接触冲量存在浮点归约量级差异。这些结果支持计算顺序与输入更新正确，但不构成任意轨迹均小于 0.16 mm 的保证。
关键 JSON 结果已随仓库归档到 [2026-09-12 基线](benchmarks/mpm_20260912/README.md)；
完整粒子快照和调试日志仍在原工作区 `artifacts/cuda_graph_20260912/`，不作为运行依赖。

## 碰撞投影计数与验证

加上 `--contact-details` 会增加独立的第三轮，统计每个碰撞体的候选检查、实际最近点/射线查询、
精确查询缓存命中、安全距离跳过、后续轮次跳过、推进次数及触及 48 次上限的次数。
计数缓冲区只在该轮启用，计数轮耗时不作为加速比。
`surface_queries` 是逻辑几何查询次数，`mesh_point_queries` / `mesh_ray_queries` 是实际执行次数；
`clearance_skips` 记录通过安全距离判断跳过的 sweep，`skipped_pass_pair_tests` 记录已证明无碰撞后
跳过的后续轮次形状检查；两者不计入实际网格查询数。输出中还记录物理模块源码哈希，便于比较不同代码版本。

碰撞投影缓存同一截面、同一形状的一次实际网格查询，包括形状局部坐标、距离和法线。
坐标完全相同时可复用原查询；坐标变化时，用已测距离减去到缓存点的位移、线缆覆盖半径和单精度舍入保护量，
得到保守净空下界。只有该下界大于“本次扫掠长度＋接触余量＋舍入保护量”时才跳过查询。
这同时保证原 sweep 可在一次迭代内无修正地到达终点，且终点在接触余量之外。
缓存锚点只由实际查询更新，不把外推下界当作新测量；查询失败的距离哨兵值不能用于安全距离判断。

若一轮中该截面对所有形状都通过包围盒排除或上述净空判断，则后续轮次也无需修正，可在同一内核中提前跳过。
任何不能确认的形状均走原始 sweep，保留 48 次上限、逐形状顺序、摩擦、APIC 修正和反作用冲量。
第一轮投影可能使截面接触另一形状，因此保留截面的完整形状遍历，不能只处理初始时相交的形状。
粒子数、频率、覆盖半径、接触余量和 `contact_iterations` 参数均保持原值，独立穿透审计继续直接查询原网格。

默认 `solve()` 会清空缓存；MPMCable 显式使用 `reuse_query_cache=True`，在刚体移动后继续复用形状局部几何。
每次求解仍用最新的旧/新刚体姿态计算局部扫掠端点，平移和旋转不会改变局部网格本身。
线缆成功重置时清空缓存。若其他调用者显式复用缓存，更改网格或几何缩放后必须更新几何数据并调用
`invalidate_query_cache()`；修改几何时不能仅依赖旧的包围盒或距离缓存。

测试默认使用仓库内冻结的参考源码，见 [test/reference](../test/reference/README.md)。
可用 `--reference` / `--reference-mpm` 覆盖为另一份源码。独立 CUDA 对照：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_contact_equivalence_cuda.py \
  --report /tmp/contact-equivalence.json
```

该测试比较平移/旋转形状、移动/旋转碰撞体、位置、速度、APIC 仿射项和反作用冲量，
另外验证真实非凸线槽内的缓存命中及失效。它与已有 `check_contacts_cuda.py`、USB 动作/重置验证配合使用。

对安全距离和轮次筛选的额外回归（参考实现须已有 `reuse_query_cache` 接口）：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_contact_acceleration_cuda.py \
  --report /tmp/contact-acceleration.json
.venv/bin/python src/dual_fr3_maniskill/test/check_trunking_contacts_cuda.py \
  --output /tmp/trunking-contact-motion
```

第一项覆盖移动点的保守复用、薄壁扫掠、终点第二轮修正、深度重叠、截面拉伸造成的半径变化、
刚体平移/旋转、多形状连续接触、40 个形状及不同投影轮次。第二项分别运行旧实现两轮、新实现一轮，
每轮执行小幅双臂运动 20 个控制步及线缆重置后 5 步，并检查原有 MPM 运行间差异。
这两项不执行完整 MTC 轨迹。

还可在实际运动过程中，对每次碰撞求解的相同输入进行 GPU 对照：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_contacts_shadow_cuda.py \
  --output /tmp/trunking-contact-shadow
```

该检查复制求解前的位置、速度、APIC 状态及累计冲量，在独立缓冲区中运行旧实现的全新网格查询，
仅让新实现的输出驱动仿真。默认覆盖 15250 次碰撞调用，误差统计留在 GPU，结束时回读；
不以这种额外求解和拷贝的耗时作为性能基准。比较器会先通过人工注入的非零误差与 NaN 验证。
它有助于区分碰撞实现差异与完整并行 MPM 独立运行的末态差异；后者仍应单独记录，不保证逐位一致。

## 右夹爪导向的 GPU 计算

右夹爪导向直接读写 MPM 当前 GPU 粒子缓冲区，质量也直接从 GPU 模型读取。
截面中心、平面交点选择、弧长渐变权重、径向投影、速度修正和反作用冲量归约均在 GPU 执行。
求解循环不再调用粒子数组的 `.numpy()` / `.assign()`，也不在循环内分配粒子临时数组。
每次求解仍重算实际交点与弧长，保持原来的 ±32 截面搜索范围、固定端排除、自由轴向滑动和渐变公式；
物理频率、粒子数、迭代数及接触参数不因这一优化改变。

PhysX 步前、步后各上传一次夹爪位置、轴向、质心及线/角速度（72 字节），
各回读一次汇总冲量、材料坐标及错误标记（32 字节），保留原有双向耦合的时序。
这不是每次 MPM 子步或纤维迭代的传输。`diagnostics()` 仍按调用需要读取状态并测量最终径向误差。
独立使用 `SlidingGuide` 时，应先 `bind(cable)`，并在夹爪状态改变后调用 `update_pose()`；
MPMCable 已在 `_update_bodies()` 中处理这一生命周期，并在交换 MPM 缓冲区后使用当前状态。

失去穿线、到达固定/自由端、非法状态会在 GPU 上保留错误标记，该次及后续导向投影停止写入粒子，
在最近的 `apply_reaction()` 刚体同步边界抛出异常。错误不会被后续有效交点或 `begin_step()` 清除，
不会继续推进下一次 PhysX 步；只有明确重置才清除标记。相较旧实现，异常回报从每次导向迭代延迟到刚体边界，
无需为检查错误增加每次导向迭代的 CPU 同步。

实现使用 ManiSkill2 配套 Warp 的单精度计算和分块弧长扫描，不依赖新版 Warp、CuPy 或新的 CUDA 上下文。
原 NumPy 实现的部分几何与归约使用双精度，因此两者不是逐位相同，须结合数值对照和运动回归验证。
下面的第一项覆盖旋转/移动夹爪、轴向滑动、分块交点、最近交点选择、MPM 缓冲区交换和错误重置；
第二项在完整线槽物理场景中对照旧 NumPy 与新 GPU 导向，执行小幅双臂运动及线缆重置：

```bash
.venv/bin/python src/dual_fr3_maniskill/test/check_guide_cuda.py \
  --output /tmp/guide-equivalence.json
.venv/bin/python src/dual_fr3_maniskill/test/check_trunking_guide_cuda.py \
  --output /tmp/trunking-guide-motion
```

第二项输出目录须尚不存在。默认旧实现运行两轮、新实现运行一轮，每轮包括 20 个运动控制步及 5 个重置后控制步；
在当前 50/500/5000 Hz 配置下，每轮共推进 0.5 秒、2500 个 MPM 子步。
旧实现的重复轮用于量化 MPM 单精度原子归约造成的运行间差异。对照同时检查最大差和均方根差，
限定在参考重复差的两倍包络及固定绝对上限之内；导向内核本身另有更严格的独立数值测试。
它不执行完整 MTC 规划或轨迹，也不代替长时间稳定性测试。

