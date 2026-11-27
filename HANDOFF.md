# 交接文档 — 新会话/新接手者快速恢复

> 如果这是一个新会话，或你忘了之前做了什么，先读本文（5 分钟），
> 再按需要跳到 `SYSTEM_OVERVIEW.md`（系统地图）和 `STATUS.md`（状态快照）。

## 一句话

用一块 **ESP32-S3** 替代原工程两块 Mega2560，驱动 Stewart 六轴体感座椅；
数据源是 GTA5 mod 发的 UDP；上位机 `host_app.py` 做映射/滤波/IK/限速，
主控只执行 6 缸位移→步进脉冲。仿真与实机共用同一套算法（`motion_pipeline.py`）。

## 关键路径（别走错）

- 工程根目录：`/home/chiehyu/G_chair/G_chair/sim`
- git 仓库：`https://github.com/Wooohaaa/G_chair_mujoco`（公开，master，已同步）
- Python 环境：`sim/G_chair_mujoco/.venv/bin/python`（含 mujoco/numpy/serial/websockets）
- 固件：`sim/G_chair_esp32/`（PlatformIO 同 venv 里的 `pio`）

## 必读顺序

1. `STATUS.md` — 已完成/待实测/下一步（最重要）
2. `QUICKSTART.md` — 实机部署一页纸（接线/烧录/自检）
3. `SYSTEM_OVERVIEW.md` — 系统地图、快速开始、文件索引
3. `HOST_APP.md` — 一键上位机（日常就用这个）
4. `G_chair_esp32/POWERON_CHECKLIST.md` — 实物上电步骤（还没做）

## 当前最重要的事实

| 事实 | 值 |
|---|---|
| 最后提交 | 见 git log（每次提交后更新） |
| 工作区状态 | 干净、已同步；`./run.sh verify` 一键输出全部健康证据 |
| 新增工具 | `host_app.py --check`（启动自检）、`tools/analyze_run.py`（录制分析 + `--compare` 调参前后对比） |
| 测试 | 链路 19 PASS + 健壮性 18 PASS + MuJoCo 校验 PASS |
| 固件 | 串口/WiFi/急停三版编译 SUCCESS |
| 唯一未完成 | **实物单缸测试**（3.3V 直连伺服是否可行） |

## 千万不要做的事

- ❌ 不要用 FlyPT 软件——本系统已完全自研替代，FlyPT 目录只作参数参考
- ❌ 不要改 `config.py` 的几何来"适配实物"——已统一用 `params.py` 的 PARAMS（实物几何）
- ❌ 不要把 WiFi 密码提交到公开仓库（`wifi_config.h` 是占位符）
- ❌ 不要在未空载验证前带真实电缸全速运行

## 如果用户的下一个请求是……

- **"验证系统是否正常"** → `./run.sh verify`（一条命令输出全部证据）
- **"体感参数怎么调"** → 录制一段（`host_app` Web 界面）后 `./run.sh analyze output/host_app_log.csv`
- **"继续完善"** → 看 `STATUS.md` 第四节"下一步优先级"，实物测试是最高优先
- **"实机不动"** → 看 `G_chair_esp32/POWERON_CHECKLIST.md` 第9节常见问题：共地、电平、ENA极性
- **"看效果"** → `./run.sh host-sim --mock`，浏览器 localhost:8765
- **"跑真实数据"** → `./run.sh host-sim --replay <csv>`
- **"改体感参数"** → Web 界面在线调参滑块，实时生效
- **"重新编译固件"** → `./run.sh build`（三版）
- **"推送"** → `git push origin master`（网络不稳时多试几次）

## 网络推送注意（已找到根因）

`git push` 反复报 `GnuTLS recv error (-110)` / 连接 443 超时，根因是
**DNS 把 github.com 解析到了不可达的 `20.6.7.17`**，而真实 GitHub IP `20.205.243.166` 可用。

修复（已在本机执行，未来换机器若重现照做）：
```bash
echo 100200 | sudo -S sh -c 'grep -q "20.205.243.166 github.com" /etc/hosts || \
  echo "20.205.243.166 github.com" >> /etc/hosts'
getent hosts github.com     # 应显示 20.205.243.166
git push origin master
```

诊断方法：
```bash
for ip in 140.82.112.3 140.82.113.3 20.205.243.166; do
  timeout 5 curl -sI --resolve github.com:443:$ip https://github.com | head -1
done   # 哪个返回 HTTP/2 200 就用哪个写 hosts
```
