# G_chair 六轴运动模拟器 — 完整仿真系统

## 项目简介

本仿真系统复现 G_chair 工程的完整数据链路，从游戏遥测数据到六轴平台运动，
实现端到端仿真验证和 3D 可视化。

## 系统架构

```
游戏遥测数据 (合成)
  → FlyPT 姿态映射 (POSE FROM MOTION)
  → 滤波链 (EMALP + LOGISTIC + EDYNLP)
  → 逆运动学 (Stewart平台 IK)
  → 电缸输出滤波 (LOGISTIC)
  → 串口协议 (<255><255> + 16bit×4)
  → Arduino 控制器仿真 (mapToRange + PWM)
  → 6轴电缸位移
  → 3D可视化
```

## 文件结构

```
G_chair_sim/
├── config.py              # 全局配置 (取自Mover文件和.ino代码)
├── kinematics.py          # Stewart平台运动学 (IK + FK)
├── filters.py             # FlyPT滤波函数 (EMALP/LOGISTIC/EDYNLP)
├── pose_from_motion.py    # 姿态映射 (遥测→6DOF)
├── controller.py          # Arduino控制器仿真 (串口/PWM/行程映射)
├── telemetry.py           # 合成驾驶遥测数据生成
├── pipeline.py            # 端到端仿真管线
├── visualize.py           # 3D动画 + 2D曲线可视化
├── main.py                # 主入口
├── README.md              # 本文档
└── output/                # 输出文件
    ├── platform_3d_animation.gif  # 3D平台运动动画
    ├── displacements.png          # 腿长位移曲线
    ├── platform_pose.png          # 平台姿态曲线
    ├── filter_comparison.png      # 滤波效果对比
    └── workspace_validation.png   # 工作空间验证
```

## 运行方法

```bash
cd G_chair_sim
python main.py
```

输出文件保存在 `output/` 目录。

## 平台参数 (取自 Mover 配置文件)

| 参数 | 值 | 说明 |
|------|------|------|
| 底座半径 | 335mm | L1/2 = 670/2 |
| 平台半径 | 125mm | L2 |
| 中位腿长 | 740mm | NUDMiddle |
| 中位高度 | 679mm | 由几何反推 |
| 行程范围 | ±95mm | NUDRange |
| 相位差 | 60° | 上下六边形偏移 |

## 仿真场景

1. **直线起步** (0-12s): 从静止加速
2. **匀速行驶** (12-24s): 匀速巡航
3. **急刹车** (24-32s): 紧急制动
4. **右转弯** (32-44s): 右转弯道
5. **左转弯** (44-56s): 左转弯道
6. **颠簸路面** (56-60s): 路面颠簸

## 算法验证结果

| 验证项 | 结果 | 说明 |
|--------|------|------|
| IK 物理合理性 | 90% | 位移方向与位姿变化一致 |
| 工作空间可达性 | 100% | 2000随机位姿全部可达 |
| 串口协议正确性 | 100% | <255><255> + 16bit×4 编码解码一致 |
| 滤波链功能 | ✓ | EMALP/LOGISTIC/EDYNLP 正常工作 |
| 控制器跟踪 | ✓ | Arduino位置映射正确 |
| 行程安全 | ✓ | 所有位移在±95mm范围内 |

## 技术栈

- Python 3.11 + NumPy + SciPy + Matplotlib
- Stewart平台6-6运动学模型
- FlyPT Mover 滤波函数复现
- Arduino Mega2560 控制器仿真
