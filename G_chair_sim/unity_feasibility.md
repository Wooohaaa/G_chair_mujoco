# G_chair 六轴运动平台 — Unity 仿真可行性评估报告

## 一、参考项目分析：full-self-surgery

参考项目 `full-self-surgery` 是一个 Unity 3D 手术仿真项目，核心架构如下：

| 技术要素 | 实现方式 |
|----------|----------|
| 引擎 | Unity 6000.6 LTS |
| 物理引擎 | SOFA（通过 SofaUnity 插件集成）+ Unity PhysX 双后端 |
| 外部通信 | HTTP JSON API（端口 8765） |
| 运动学 | DLS-IK（阻尼最小二乘逆运动学） |
| 场景构建 | 全程序化生成（无预制件依赖） |
| 截图 | 2×超采样 + 8×MSAA + GPU降采样，最高8K |
| 多机位 | 6宫格视频墙（主/前/后/左/右/顶） |

### 关键可复用架构模式

```
Python 后端 (仿真引擎)
  → HTTP JSON API (port 8765)
  → Unity 前端 (3D可视化 + HUD)
```

- Python 端运行仿真逻辑（遥测→滤波→IK→控制器）
- Unity 端接收 JSON 数据，实时渲染 3D 模型
- 单向数据流：Python → Unity（无需双向通信）
- 50Hz 数据发送，Unity 60fps 渲染

---

## 二、Unity 实现 G_chair 仿真的可行性评估

### 总评：高度可行 ✅

G_chair 仿真比参考项目**简单得多**：
- 无软组织 FEM（不需要 SOFA）
- 无多臂协调（只有 6 个直线执行器）
- 无手术工具（只有座椅+平台）
- 无安全协议（纯运动学仿真）

### 逐项评估

#### 1. 3D 模型精度 — ✅ 可行

**目标**：模型跟实物一样

**方案**：
- 从工程文件中导入 DWG/STL 图纸（座椅固定板、电缸固定板、鱼眼固定板、3D打印件）
- 用 Unity 基本体构建：铝型材框架、伺服电机、滚珠丝杠、电缸外壳
- 座椅：可用通用赛车座椅模型或自行建模
- 材质：PBR 材质（金属度、粗糙度、法线贴图）
- 电缸：圆柱体 + 丝杠纹理 + 伺服电机细节

**工作量**：2-3 天（含材质调优）

#### 2. 实时运动学 — ✅ 可行

**方案**：
- Python 端运行完整 IK（已实现并验证）
- 发送 6 轴位移 + 平台位姿到 Unity
- Unity 端直接更新 6 根电缸的伸缩长度 + 平台 transform
- 不需要 Unity 端 IK（Python 端已计算）
- 可选：在 Unity 端也实现 IK 做双重验证

**数据流**：
```
Python IK → 6轴位移(mm) → JSON → Unity → 更新6个执行器 + 平台位置/旋转
```

**工作量**：1 天（C# 数据接收 + 模型更新脚本）

#### 3. 物理仿真 — ✅ 可行（两种方式）

**方案 A：纯运动学（推荐）**
- 不需要物理引擎
- 直接根据 IK 结果设置 6 根电缸的 localScale（伸缩长度）
- 平台 transform 由 6 根电缸位置决定
- 优点：精确、无延迟、无物理冲突
- 适用：展示运动效果

**方案 B：Unity PhysX 约束**
- 用 ArticulationBody 约束 6 根电缸
- 平台用 ArticulationBody 约束到 6 根电缸
- 优点：有物理碰撞、质量、惯性
- 缺点：约束求解可能不精确、有抖动
- 适用：需要物理交互的场景

**推荐**：方案 A（纯运动学），简单精确

#### 4. 实时数据流可视化 — ✅ 可行

**方案**：
- Unity Canvas UI 显示数据流面板
- 仿照参考项目的 HUD 风格
- 显示：遥测数据、姿态、位移、数据流管线图
- Canvas 叠在 3D 场景上方

**工作量**：1 天

#### 5. 视觉品质 — ✅ 可达高水准

| 效果 | 技术方案 |
|------|----------|
| 金属质感 | PBR 材质（Metallic=1, Roughness=0.3-0.7） |
| 电缸细节 | 程序化建模（电机+丝杠+端部铰接） |
| 地面反射 | 反射探针 + 平面反射 |
| 阴影 | 实时阴影（Directional Light + Shadowmask） |
| 环境 | HDR 环境贴图 + 后处理（Bloom, AO, Color Grading） |
| 座椅 | 导入 3D 模型或程序化建模 |
| 灯光 | 主光源 + 补光 + 环境光 |

**工作量**：1-2 天（含材质和灯光调优）

#### 6. 外部控制 — ✅ 可直接复用

- Python 后端发送 HTTP JSON（端口 8765）
- Unity C# 端用 HttpClient 接收
- 数据格式与当前 Python 仿真一致

---

## 三、架构设计

### 系统架构图

```
┌─────────────────────────────────────────────────────────┐
│                   Python 仿真后端                        │
│                                                         │
│  遥测生成 → 姿态映射 → 滤波链 → IK → 控制器模拟           │
│     ↓           ↓        ↓       ↓        ↓             │
│  25通道     6DOF     LOGISTIC  6轴位移  PWM频率          │
│                                                         │
│  → HTTP POST http://localhost:8765/data (50Hz)          │
└─────────────────────┬───────────────────────────────────┘
                      │ JSON
┌─────────────────────┴───────────────────────────────────┐
│                    Unity 3D 前端                         │
│                                                         │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────┐  │
│  │ HTTP数据接收 │ →  │ 平台控制器    │ →  │ 3D渲染    │  │
│  │ HttpClient  │    │ PlatformCtrl │    │ Camera    │  │
│  └─────────────┘    └──────────────┘    └───────────┘  │
│                          ↓                              │
│                   ┌──────────────┐                      │
│                   │ 6轴电缸更新   │                      │
│                   │ ActuatorSync │                      │
│                   └──────────────┘                      │
│                         ↓                                │
│              ┌────────────────────────────┐              │
│              │ 3D模型: 底座+6电缸+平台+座椅 │              │
│              │ PBR材质 + 实时阴影 + 后处理  │              │
│              └────────────────────────────┘              │
│                         ↓                                │
│              ┌────────────────────────────┐              │
│              │ Canvas UI (数据流面板)      │              │
│              │ 遥测/姿态/位移/数据流图      │              │
│              └────────────────────────────┘              │
└───────────────────────────────────────────────────────────┘
```

### 3D 模型层级结构

```
StewartPlatform (空物体, 原点在底座中心)
├── BasePlate (底座六边形板)
│   ├── 6个 BaseJoint_0~5 (底座铰接点, 蓝色球体)
│   └── 铝型材框架 (程序化建模)
├── Actuator_0~5 (6根电缸, 每根包含)
│   ├── MotorBody (伺服电机外壳, 灰色圆柱)
│   ├── Screw (滚珠丝杠, 银色圆柱)
│   ├── Rod (活塞杆, 黑色圆柱)
│   ├── TopJoint (顶部铰接, 球关节)
│   └── BottomJoint (底部铰接, 球关节)
├── PlatformPlate (平台六边形板)
│   ├── 6个 PlatJoint_0~5 (平台铰接点, 绿色球体)
│   └── 座椅安装座
├── Seat (座椅)
│   ├── SeatBase (座垫)
│   ├── SeatBack (靠背)
│   └── SteeringWheel (方向盘, 可选)
└── Environment
    ├── Floor (地面)
    ├── Lights (灯光)
    └── PostProcessing (后处理)
```

### 电缸运动更新逻辑

```csharp
// 每根电缸的更新
void UpdateActuator(ActuatorModel actuator, float displacement_mm) {
    // displacement_mm: 正=伸长, 负=缩短
    // 基础长度 + 位移 = 当前长度
    float currentLength = baseLength + displacement_mm / 1000f; // 转为米
    
    // 更新电缸模型
    actuator.rod.localScale.y = currentLength / baseRodLength;
    
    // 更新活塞杆位置
    actuator.rod.localPosition.y = currentLength / 2f;
    
    // 更新顶部铰接位置
    actuator.topJoint.localPosition.y = currentLength;
}

// 平台更新
void UpdatePlatform(float sway, float surge, float heave, 
                    float roll, float pitch, float yaw) {
    // 直接设置平台 transform
    platformPlate.transform.localPosition = 
        new Vector3(sway/1000f, heave/1000f, surge/1000f);
    platformPlate.transform.localRotation = 
        Quaternion.Euler(roll, yaw, pitch);
}
```

---

## 四、实现路线图

### 阶段 1：Unity 项目搭建 + 3D 模型（2-3天）
1. 安装 Unity 6 LTS，创建项目
2. 导入/构建 Stewart 平台 3D 模型
3. 设置 PBR 材质、灯光、阴影
4. 验证模型静态渲染效果

### 阶段 2：数据通信 + 运动更新（1天）
1. C# HttpClient 接收 Python 端 JSON 数据
2. PlatformController 更新平台 transform
3. ActuatorSync 更新 6 根电缸伸缩
4. 验证实时运动效果

### 阶段 3：数据流 HUD（1天）
1. Canvas UI 面板（遥测、姿态、位移）
2. 数据流管线图（仿参考项目风格）
3. 场景标签、时间显示、重置按钮

### 阶段 4：视觉增强（1-2天）
1. 后处理（Bloom、AO、Color Grading）
2. 多机位切换
3. 电缸运动动画（伺服电机转动、丝杠伸缩）
4. 音效（可选：电机嗡嗡声、运动风声）

### 阶段 5：整合测试（1天）
1. Python 后端 + Unity 前端联调
2. 性能优化（确保 60fps）
3. 视觉效果调优

**总工期估算**：6-8 天

---

## 五、风险与挑战

| 风险 | 影响 | 应对 |
|------|------|------|
| 3D 模型精度不足 | 视觉效果差 | 从工程 DWG 文件导入；必要时用 Blender 建模 |
| Unity 安装/配置复杂 | 开发延迟 | Unity Hub 一键安装；参考项目已验证 Unity 6 兼容性 |
| 数据同步延迟 | 运动不同步 | Python 50Hz 发送；Unity 端插值补帧 |
| 6 根电缸约束精度 | 平台位置偏差 | 方案 A（纯运动学）避免约束问题 |
| 材质/灯光调优耗时 | 进度延迟 | 先用标准材质快速出效果，再逐步提升 |

---

## 六、与当前 Python 方案的对比

| 维度 | Python + Canvas (当前) | Unity 3D (新方案) |
|------|----------------------|-------------------|
| 3D 渲染 | Canvas 2D 投影 | GPU 3D 渲染 + PBR |
| 模型精度 | 抽象线条 | 物理级模型 |
| 视觉效果 | 基础 | 电影级（阴影/反射/后处理） |
| 交互性 | 鼠标拖拽旋转 | Orbit Camera + 多机位 |
| 开发周期 | 1天 | 6-8天 |
| 运行性能 | 取决于浏览器 | 60fps GPU 渲染 |
| 可扩展性 | 有限 | 高（可加物理/音效/多机位） |

---

## 七、结论

**用 Unity 实现 G_chair 仿真高度可行**，参考项目 `full-self-surgery` 已验证：
- Unity 6 + HTTP API 架构成熟可靠
- 程序化场景生成可行
- 实时 3D 渲染 + 数据流可视化可同时运行

G_chair 比手术仿真简单得多（无 FEM、无多臂、无手术工具），核心工作是：
1. 构建 Stewart 平台 3D 模型（2-3天）
2. C# 数据接收 + 运动更新（1天）
3. HUD 数据流面板（1天）
4. 视觉增强（1-2天）

预计 6-8 天可完成一个视觉效果接近实物运行的高保真仿真。
