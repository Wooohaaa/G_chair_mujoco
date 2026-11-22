# G_chair 六轴 Stewart 平台 · MuJoCo 仿真

不接触实物即可验证 G_chair 的电缸控制链、位姿映射、行程与奇异位形。

```
FlyPT/遥测 ──> 6DOF 位姿 ──> 逆运动学 IK ──> 6 缸位移 ──> MuJoCo 电动缸+闭环平台
                                                     │
                                     实际平台位姿 <──┘  (正运动学/FK 校验)
```

## 快速开始

```bash
cd sim/G_chair_mujoco
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt

# 打开交互查看器, 播放演示动作 (升沉/横滚/俯仰/偏航/混合)
./.venv/bin/python sim.py --viewer

# 离屏渲染视频 (无需显示器, 已验证)
./.venv/bin/python sim.py --scenario demo --duration 24 --render output/demo.mp4

# 静态位姿校验 + 图表
./.venv/bin/python sim.py --validate
```

> 已生成的样例见 `output/`：`demo.mp4`、`snapshot.png`、`leg_layout.png`、
> `validation.png`、`sim_tracking.png`。

## 常用命令

| 命令 | 说明 |
|------|------|
| `sim.py --viewer` | MuJoCo 交互查看器, 实时播放 |
| `sim.py --scenario roll --viewer` | 单独测试横滚 (`neutral/heave/roll/pitch/yaw/sway/surge/combined/demo`) |
| `sim.py --render out.mp4 --duration 20` | 离屏渲染 mp4/gif |
| `sim.py --scenario combined --plot --save-csv` | 保存跟踪曲线与 CSV 日志 |
| `sim.py --pipeline --viewer` | 接入原 `G_chair_sim` 遥测→姿态→滤波链 |
| `sim.py --validate` | 极值位姿静态跟踪、工作空间、机构一致性校验 |
| `sim.py --realtime` | 无查看器时按真实时间播放 |

## 模型

- 6 缸电动缸：每条缸 = 底座万向节(2 hinge) + 直线滑轨(slide) + 两端球头。
- 上平台为 `free joint` 的 6 自由度刚体，6 个 `equality/connect` 把缸顶约束到平台铰点，
  构成真实的闭环 Stewart 机构（不是把位姿直接“贴”到平台上的纯动画）。
- 每根缸由一个 `position` 执行器驱动，控制量 = 相对中位的位移（m），
  与真实控制器输出的 ±95 mm 行程一一对应。

| 参数 | 值 | 来源 |
|------|----|------|
| 底座铰点半径 | 0.540 m | 图纸六边形底座 (~1.1 m) |
| 平台铰点半径 | 0.300 m | 座椅固定板 (~0.6 m) |
| 中位腿长 | 0.740 m | Mover `NUDMiddle` |
| 中位平台高度 | 0.664 m | 由腿长几何反推 |
| 电缸行程 | 0.192 m (±95 mm) | `servoController.ino` `xingcheng` |
| 铰点布置 | 3 组“对”铰点：**底座后方正中间(+Y=90°)一组，左右 210°/330° 对称**；平台错位 60° | 成品照片（2026-11 从旧 0/120/240 整体旋转 90° 修正）；底座 12°/平台 16° 半张角 |

### 为什么改了旧仿真的半径/角度

旧 `G_chair_sim` 把 `NUDRigL1=670` 当作直径得到 `r_base=335mm`，且上下铰点角度完全重合。
该圆形对称几何的机构雅可比**奇异**（条件数 ~1e18）：平台偏航/升降方向零刚度，
在 MuJoCo 里受重力会直接“塌陷”成交叉腿的错误装配分支（实测平台偏航 90°、高度掉 96mm）。

本项目改用**非奇异的 3 组对铰点**布置，并选择唯一同时满足
「支腿不交叉 + 6 腿等长 + 雅可比良态」的配对（条件数 **7.7**）。
几何默认值与实物底座/座椅板尺寸相符，可在 `params.py` 中按你的实际测量值修改后
重新 `python build_model.py` 生成 `model.xml`。

## 外观（2026-11 增强）

已按成品照片把抽象模型改成接近实物的外观：铝型材框架底座/平台、黑色缸筒 +
银色活塞杆 + 伺服电机盒、桶形赛车座椅（侧翼/肩翼/头枕）、方向盘与显示器支架。
这些**全部是纯视觉 geom**——不参与碰撞、不改 inertial/joint/site/equality/actuator，
所以运动学与跟踪精度完全不变（校验仍 PASS）。全景图见 `output/appearance_full.png`。
重新生成：`python build_model.py`。

## 校验结果（`python sim.py --validate`）

| 项目 | 结果 |
|------|------|
| 机构雅可比条件数 | 7.73（远离奇异） |
| 包线内随机位姿可达率 | 500/500 = 100% |
| 随机最大缸位移 | 93.1 mm（行程 ±95 mm 内） |
| 极值位姿静态跟踪 | 位置 RMS 1.60 mm / MAX 2.02 mm；姿态 RMS 0.068° / MAX 0.126° |
| MuJoCo 实际位姿 vs 解析 FK | 位置 MAX 0.25 mm，姿态 MAX 0.016° |

包线取自 Mover 配置：sway/surge ±50 mm、heave ±30 mm、roll/pitch ±6°、yaw ±8°。

## 文件

```
G_chair_mujoco/
├── params.py        # 几何/质量/执行器参数（改这里定制）
├── gchair_kin.py    # 通用铰点 Stewart IK/FK
├── build_model.py   # 参数化生成 model.xml
├── model.xml        # 由 build_model.py 生成
├── sim.py           # 主程序: 场景/驱动/查看/渲染/日志
├── validate.py      # 校验与图表
├── requirements.txt
└── output/          # 视频/曲线/校验报告
```

## 说明与限制

- 这是**机构级**仿真：验证 IK/FK、闭环几何、行程、奇异、控制跟踪。
- 未建模真实伺服的延迟/报警、丝杆间隙、鱼眼旷量、铝型材/座椅结构柔度、
  摩擦与负载扰动；这些需在实物上最终确认。
- **关于"缸杆脱离底座"**：`check_gap.py` 实测 30s 视频驱动下 6 条缸的
  tip↔plat 间隙 MAX 1.18mm / 平均 0.17mm，物理约束没有失效。视觉上像脱离，
  是因为原模型底座只有一块平圆盘、没有铰点安装座。已在 `build_model.py` 里
  给每个底座铰点加上球碗座（`socket_cup_*` / `socket_ball_*` 纯视觉 geom，
  不影响动力学），重新 `python build_model.py` 生成 `model.xml` 即可。
  若要看真实间隙数据：`./.venv/bin/python check_gap.py [compare_log.csv]`。
- `--pipeline` 复用 `../G_chair_sim` 的遥测、姿态映射与滤波链，只把最后的
  IK 换成当前模型，用来看完整数据链在正确机构上的表现。
- 查看器打不开时（远程/无显示），用 `--render` 离屏渲染；
  必要时设置 `MUJOCO_GL=egl`（有 GPU）或 `MUJOCO_GL=osmesa`。
