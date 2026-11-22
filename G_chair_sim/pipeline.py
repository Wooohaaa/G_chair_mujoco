"""
pipeline.py — 端到端仿真管线
串联: 遥测 → 姿态映射 → 滤波 → 逆运动学 → 串口协议 → Arduino控制器
"""

import numpy as np
from config import RIG, SIM, FILTERS
from kinematics import StewartPlatform
from filters import FilterChain
from pose_from_motion import PoseFromMotion
from controller import DualBoardController
from telemetry import TelemetryGenerator


class SimulationPipeline:
    """完整的 G_chair 运动模拟器仿真管线"""

    def __init__(self):
        self.rig = StewartPlatform(
            r_base=RIG['r_base'],
            r_plat=RIG['r_plat'],
            leg_middle=RIG['leg_middle'],
            height=RIG['height'],
            range_mm=RIG['range'],
            angle_offset_deg=RIG['angle_offset'],
        )
        self.pose_mapper = PoseFromMotion()
        self.filter_chain = FilterChain(FILTERS)
        self.controller = DualBoardController()
        self.telemetry_gen = TelemetryGenerator(dt=SIM['dt'])
        self.dt = SIM['dt']

        # 记录历史数据 (用于可视化)
        self.history = {
            'time': [],
            'telemetry': [],
            'pose_raw': [],
            'pose_filtered': [],
            'displacements': [],
            'arduino_positions': [],
            'serial_frames': [],
            'pwm_freqs': [],
            'scenario': [],
        }

    def run_step(self):
        """运行一个仿真步"""
        # 1. 生成遥测数据
        telemetry = self.telemetry_gen.generate()

        # 2. 姿态映射 (遥测 → 6 DOF)
        pose_raw = self.pose_mapper.compute(telemetry, self.dt)

        # 3. 滤波 (悬挂滤波 + 旋转滤波)
        pose_filt = self.pose_mapper.apply_filters(
            pose_raw, self.filter_chain, self.dt
        )

        # 4. 逆运动学 (姿态 → 腿长位移)
        # pose_filt 中的 sway/surge/heave 是 mm, yaw/roll/pitch 是度
        x = pose_filt['sway'] / 1000.0  # mm → m (IK内部用mm)
        y = pose_filt['surge'] / 1000.0
        z = pose_filt['heave'] / 1000.0
        roll = np.radians(pose_filt['roll'])
        pitch = np.radians(pose_filt['pitch'])
        yaw = np.radians(pose_filt['yaw'])

        # IK 内部使用 mm 单位
        x_mm = pose_filt['sway']
        y_mm = pose_filt['surge']
        z_mm = pose_filt['heave']
        disp = self.rig.inverse_kinematics(
            x_mm, y_mm, z_mm, roll, pitch, yaw
        )

        # 5. 电缸输出滤波 LOGISTIC(VALUE;70;0.9)
        disp_filtered = self.filter_chain.filter_actuator(disp)

        # 6. 限幅到行程范围
        disp_filtered = np.clip(disp_filtered, -RIG['range'], RIG['range'])

        # 7. Arduino 控制器仿真 (多步快速推进)
        num_ctrl_steps = int(0.05 / self.dt)  # 50ms 控制周期
        for _ in range(max(num_ctrl_steps, 1)):
            self.controller.step(disp_filtered, dt=self.dt * 0.01)

        # 8. 获取控制器物理位置
        ctrl_disp = self.controller.get_all_displacements()

        # 9. 构建串口帧 (验证协议)
        frame1, frame2 = self.controller.get_serial_frames(disp_filtered)

        # 记录历史
        self.history['time'].append(self.telemetry_gen.t)
        self.history['telemetry'].append(telemetry.copy())
        self.history['pose_raw'].append(pose_raw.copy())
        self.history['pose_filtered'].append(pose_filt.copy())
        self.history['displacements'].append(disp_filtered.copy())
        self.history['arduino_positions'].append(ctrl_disp.copy())
        self.history['serial_frames'].append((frame1, frame2))
        self.history['pwm_freqs'].append(
            list(self.controller.board1.get_pwm_frequencies()) +
            list(self.controller.board2.get_pwm_frequencies())
        )
        self.history['scenario'].append(telemetry['_scenario'])

    def run(self, num_steps=None):
        """运行完整仿真"""
        if num_steps is None:
            num_steps = int(SIM['duration'] / self.dt)

        for i in range(num_steps):
            self.run_step()

        return self.history

    def get_platform_pose_at(self, idx):
        """获取某个时刻的平台位姿 (用于3D可视化)"""
        pose = self.history['pose_filtered'][idx]
        return pose

    def get_displacements_at(self, idx):
        """获取某个时刻的腿长位移"""
        return self.history['displacements'][idx]

    def validate_ik_fk(self, num_tests=50):
        """验证 IK 物理合理性 + FK 残差验证"""
        ik_physics_ok = 0
        ik_physics_total = 0
        fk_residuals = []

        for _ in range(num_tests):
            x = np.random.uniform(-20, 20)
            y = np.random.uniform(-20, 20)
            z = np.random.uniform(-10, 10)
            roll = np.random.uniform(-0.08, 0.08)
            pitch = np.random.uniform(-0.08, 0.08)
            yaw = np.random.uniform(-0.08, 0.08)

            disp = self.rig.inverse_kinematics(x, y, z, roll, pitch, yaw)

            # IK 物理合理性
            ik_physics_total += 1
            if np.all(np.abs(disp) <= RIG['range']):
                if z > 0 and np.all(disp > 0):
                    ik_physics_ok += 1
                elif z < 0 and np.all(disp < 0):
                    ik_physics_ok += 1
                elif z == 0 or np.sign(np.mean(disp)) == np.sign(z):
                    ik_physics_ok += 1

            # FK 残差验证
            if np.any(np.abs(disp) > RIG['range']):
                continue
            pose_fk = self.rig.forward_kinematics(disp)
            disp_fk = self.rig.inverse_kinematics(*pose_fk)
            residual = np.max(np.abs(disp - disp_fk))
            fk_residuals.append(residual)

        result = {
            'ik_physics_ok': ik_physics_ok,
            'ik_physics_total': ik_physics_total,
            'ik_physics_ratio': ik_physics_ok / ik_physics_total if ik_physics_total > 0 else 0,
        }
        if fk_residuals:
            result['fk_max_residual'] = max(fk_residuals)
            result['fk_mean_residual'] = np.mean(fk_residuals)
            result['fk_num_tests'] = len(fk_residuals)
            result['fk_pass_rate'] = sum(1 for r in fk_residuals if r < 0.1) / len(fk_residuals)
        return result

    def validate_serial_protocol(self, num_tests=20):
        """验证串口协议正确性: 编码→解码往返一致性"""
        results = []
        for _ in range(num_tests):
            disp = np.random.uniform(-90, 90, 6)

            # 构建串口帧
            frame1, frame2 = self.controller.get_serial_frames(disp)

            # 验证帧格式
            assert frame1[0] == 255 and frame1[1] == 255, "帧1起始标记错误"
            assert frame2[0] == 255 and frame2[1] == 255, "帧2起始标记错误"

            # 解析回来 (直接验证编码/解码一致性)
            b1_targets = self.controller.board1.parse_serial(frame1)
            b2_targets = self.controller.board2.parse_serial(frame2)

            if b1_targets is None or b2_targets is None:
                results.append(False)
                continue

            # 将 Arduino 目标位置转为物理位移
            b1_disp = [self.controller.board1.physical_displacement(t) for t in b1_targets]
            b2_disp = [self.controller.board2.physical_displacement(t) for t in b2_targets]

            # 组合: 板1控轴1-4, 板2控轴5-6
            ctrl_disp = np.array([b1_disp[0], b1_disp[1], b1_disp[2], b1_disp[3],
                                  b2_disp[0], b2_disp[1]])

            # 由于 dispatch_displacements 只将轴5/6发给板2, 其余填0
            # 所以板2的物理位移对于轴3/4是0(中位), 这是预期行为
            err = np.max(np.abs(ctrl_disp - disp))
            results.append(err < 3.0)  # 允许3mm量化误差

        return {
            'pass_rate': sum(results) / len(results),
            'num_tests': len(results),
        }
