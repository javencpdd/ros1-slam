# regression/ 三分类登记表

2026-11-18（计划 M8）。55 个 `.py` 平铺在此目录，此前只能靠 docstring 猜用途。
本表把「哪个文件干哪类事」变成可查的事实。

## 为什么不搬文件

`note/04`、`note/05` 是逐条登记制，大量条目直接写文件名（如「R882 a5_fourcase」）。
重命名或移入子目录会让这批引用变悬空，而台账的历史准确性比目录整齐更要紧。
所以本表是**登记**不是**搬迁**——目录结构不变，分类信息在表里。

## 三类 + 测试

| 类 | 定义 | 有什么特征 |
|---|---|---|
| **执行器**（runtime cell） | 起 ROS/Gazebo/MuJoCo，跑一个场景，产出证据目录 | 需要运行域，不能裸跑 |
| **探针** | 只读观测既有节点/数据流，产读数，不改变系统 | 也需运行域，但不产出「通过/失败」判决 |
| **生成器** | 构造夹具、扰动轨、静态障碍、地图 | 纯输入构造，多数可裸跑 |
| **测试**（`test_*.py`） | 宿主侧 unittest，锁判据与纯函数 | 不需要 ROS，可直接 `python3 -m unittest` |

## 执行器（31 个，需运行域）

### A1–A2 组合与故障注入
| 文件 | 干什么 |
|---|---|
| `a1_combo_matrix.py` | A1 全组合矩阵 runner（预登记） |
| `a1_3d_combo_matrix.py` | 3D 装配场景矩阵：平地夹具 + NIST 台阶，重复多轮 |
| `a2_gazebo_stop.py` | Gazebo 崩溃源 / supervisor hold → 物理停止探针 |
| `a2_process_lifecycle.py` | OS 进程故障注入：崩溃、就绪/运行超时、清理 |
| `a2_stop_chain.py` | 停止链运行面：mux hold→零速、释放恢复、源超时→零速 |

### A3–A4 接口与规划运行面
| 文件 | 干什么 |
|---|---|
| `a3_runtime.py` | A3 ROS1 接口证据：同 XY 双层、净空、free/unknown、无支撑与单向不可达 |
| `a3d_algorithm_matrix.py` | 3D 算法级矩阵：全局 ROS-vs-设计、局部 plan-vs-评估 |
| `a45_3d_test.py` | 3D 导航测试：多层面支撑图全局规划（A4）+ 运动学局部执行（A5 设计层） |
| `a4_runtime.py` | A4 运行面：把 support_planner 作为 ROS1 节点实跑（真值/扰动双轨） |

### A5 执行与跨层
| 文件 | 干什么 |
|---|---|
| `a5_cross_layer_runtime.py` | A5 运动学台阶跨层探针：五层支撑、无物理 |
| `a5_gazebo_runtime.py` | 原生局部规划器 → 显式门控平地载体 → Gazebo 物理 |
| `a5_gazebo3d_runtime.py` | A5 ROS 局部规划 → 缩放 NIST 世界上的 Gazebo 运动学 3D 载体 |
| `a5_v8x1_runtime.py` | A5 ROS 局部节点 → V8-X1 MuJoCo 策略链，NIST 台阶 |
| `a5_v8x1_stop_smoke.py` | ROS 局部节点 → V8-X1 270 策略，平地 + 感知阻塞 |
| `a5_v8x1_replay.py` | 回放录制的 V8-X1 桥指令轨迹，定 tick 比对 |
| `a5_v8x1_top_hold.py` | 给录制的 V8-X1 爬升轨迹加「保持」段并测量 |

### A6 定位
| 文件 | 干什么 |
|---|---|
| `a6_runtime.py` | 定位消息失效/重定位的 ROS1 下游撤销与恢复证据 |
| `a6_backend_compare.py` | 单条实时传感器流上的精度对比 |
| `a6_ndt_replay.py` | bag 回放冻结传感器流时录制 NDT monitor 输出 |

### A7 执行与跟踪
| 文件 | 干什么 |
|---|---|
| `a7_exec_runtime.py` | A7 3D 执行运行面：local_native ROS 节点 + 运动学载体 |
| `a7_runtime_matrix.py` | A7 规划运行矩阵：10 场景 × 真值/估计，经 support_planner |
| `a7_tracking_runtime.py` | SC11 ROS 跟踪运行：参考、干净跟踪、横向偏差 |

### 定位冒烟（hdl 系列）
| 文件 | 干什么 |
|---|---|
| `hdl_global_live_smoke.py` | 实时 Gazebo 扫描 → hdl_global_localization 查询 → 候选 vs 真值 |
| `hdl_global_consensus_smoke.py` | 实时扫描 → 三 BBS 分辨率 → 候选共识 → Gazebo 验证 |
| `hdl_global_map_smoke.py` | hdl_global_localization + m1 合成 PCD 冒烟 |
| `hdl_native_relocalize_smoke.py` | 原生 hdl /relocalize 冒烟（m1 世界） |
| `nist_stairs_runtime.py` | NIST 台阶运动学运行探针（mesh 派生支撑图） |

### 主链与批量
| 文件 | 干什么 |
|---|---|
| `boundary_chain_runtime.py` | 边界主链冒烟：mux → /cmd_vel → UDP 桥 → Gazebo 端点 |
| `gazebo3d_live_map.py` | 实时消费传感器建成的支撑图：版本切换、位姿噪声、障碍 |
| `regression.py` | V8-13 双轨闭环与组合回归矩阵（R658） |
| `runtime_batch.py` | 预登记后端实验后执行，不丢失败 |

## 探针（2 个，只读数不判决）

| 文件 | 干什么 |
|---|---|
| `hdl_runtime_probe.py` | 用隔离的、真值适配的传感器测既有 HDL nodelet |
| `runtime_evaluator.py` | 独立运行证据评价器：只读观测与注入记录，**不导入**被测门控器/规划器 |

## 基础设施（1 个）

| 文件 | 干什么 |
|---|---|
| `runtime_lab.py` | 拥有隔离 ROS master 与可选 Gazebo 栈，始终回收子进程 |

## 生成器（4 个，构造输入）

| 文件 | 干什么 | 可裸跑 |
|---|---|---|
| `estimated_track.py` | 有界、可复现的 map→odom 扰动轨（估计定位用） | 是 |
| `global_obstacles.py` | 派生 SC02/05/06 静态箱体用例与独立连续路线 | 是 |
| `noisy_localization.py` | 把种子化扰动轨作为动态 map→odom 发布 | 否（要 ROS） |
| `gazebo3d_mapping.py` | Gazebo 激光 + 里程 → 缩放 NIST 台阶的支撑/高程图 | 否（要 Gazebo） |

## 测试（17 个，宿主可跑，`python3 -m unittest`）

锁判据与纯函数，**不需要 ROS**——这是 CI 与本地快速回归的主力面。

```
test_a3_runtime.py            test_a5_gazebo3d_runtime.py   test_a7_exec_runtime.py
test_a3d_algorithm_matrix.py  test_a5_gazebo_runtime.py      test_a7_runtime_matrix.py
test_a4_runtime.py            test_a5_v8x1_bridge.py         test_boundary_endpoint.py
test_a5_cross_layer_runtime.py test_a5_v8x1_stop_smoke.py    test_estimated_track.py
test_a6_runtime.py            test_gazebo3d_mapping.py       test_nist_stairs_runtime.py
test_regression.py            test_runtime_lab.py
```

## 跑法

```bash
# 全部宿主测试（不需要容器）
python3 scripts/dev/dev.py test ws/src/90_platform/v8_platform/regression

# 单个执行器（需要运行域，容器内）
docker compose -f docker/docker-compose.yml --env-file docker/.env exec -T sim-viz \
  bash -lc 'source /opt/ros/noetic/setup.bash && source ws/devel/setup.bash && \
  python3 ws/src/90_platform/v8_platform/regression/<名字>.py --help'
```

## 维护约定

- 新增文件请在上面某类表里补一行；**不要**为分类而移动既有文件（见开头「为什么不搬文件」）。
- 分类以「产出什么」为准：产出**证据目录**＝执行器；产出**读数**＝探针；产出**输入**＝生成器。
- 一个文件跨两类时，按它被引用的主要用途归一类，并在说明里提另一面
  （如 `gazebo3d_mapping.py` 既是执行器也是生成器，本表按「产出地图」归生成器）。
