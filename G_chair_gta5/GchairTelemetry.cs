// ============================================================================
// GchairTelemetry.cs — GTA5 -> G_chair UDP 遥测发送脚本 (Community SHVDN v3)
//
// 把玩家当前车辆的运动状态按 "25 个空格分隔浮点数" 发到仿真PC,
// 格式与工程自带 DCS_To_FlyPT_Mover.lua / udp_telemetry.py 完全一致。
//
// 安装: 见同目录 README.md
// 注意: 只能用于故事模式 (ScriptHookV 在 GTA Online 会关闭游戏)。
// ============================================================================

using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using GTA;

namespace GchairTelemetry
{
    public class GchairTelemetry : Script
    {
        // ======== 按你的网络改这里 ========
        private const string TargetIp = "10.148.82.176";  // 仿真PC 的 IP
        private const int TargetPort = 4124;              // 与 sim_udp.py 的 --port 一致

        private readonly UdpClient _udp = new UdpClient();
        private readonly CultureInfo _inv = CultureInfo.InvariantCulture;
        private DateTime _last;
        private DateTime _start;
        private Vector3 _prevVel;
        private float _qx, _qy, _qz, _qw;
        private bool _hasPrev;

        public GchairTelemetry()
        {
            _last = DateTime.Now;
            _start = _last;
            Tick += OnTick;
            Aborted += (s, e) => _udp.Close();
        }

        private void OnTick(object sender, EventArgs e)
        {
            // 用真实墙上时间做差分: GTA 帧率会波动, 不能用固定步长
            DateTime now = DateTime.Now;
            float dt = (float)(now - _last).TotalSeconds;
            _last = now;
            if (dt <= 0f || dt > 0.5f) dt = 1f / 60f;   // 首帧/卡顿保护

            float[] ch = new float[25];
            ch[24] = (float)(now - _start).TotalSeconds;

            Vehicle v = Game.Player.Character.CurrentVehicle;
            if (v != null && v.Exists())
            {
                Vector3 vel = v.Velocity;          // 世界系速度 m/s
                var q = v.Quaternion;
                float qx = q.X, qy = q.Y, qz = q.Z, qw = q.W;

                if (_hasPrev)
                {
                    // 世界系加速度 -> 车体轴, 转 G
                    Vector3 aw = new Vector3(
                        (vel.X - _prevVel.X) / dt,
                        (vel.Y - _prevVel.Y) / dt,
                        (vel.Z - _prevVel.Z) / dt);

                    Vector3 right = v.RightVector;
                    Vector3 fwd = v.ForwardVector;
                    Vector3 up = v.UpVector;

                    // DCS 通道约定: 0=横向(右), 1=纵向(前), 2=竖直(上), 单位 G
                    ch[0] = Dot(aw, right) / 9.81f;   // accel_x
                    ch[1] = Dot(aw, fwd) / 9.81f;     // accel_z
                    ch[2] = Dot(aw, up) / 9.81f;      // accel_y

                    ch[3] = Dot(vel, right);          // speed_x
                    ch[4] = Dot(vel, fwd);            // speed_z
                    ch[5] = Dot(vel, up);             // speed_y

                    // 角速度: 由四元数差分求世界系 omega, 再投影到车体轴
                    // DCS: rot_x=roll(绕车头), rot_y=yaw(绕竖直), rot_z=pitch(绕横向)
                    Vector3 wWorld = AngularRate(_qx, _qy, _qz, _qw, qx, qy, qz, qw, dt);
                    ch[7] = Dot(wWorld, fwd);         // rot_x  roll
                    ch[6] = Dot(wWorld, up);          // rot_y  yaw
                    ch[8] = Dot(wWorld, right);       // rot_z  pitch
                }

                ch[9]  = v.Heading * (float)Math.PI / 180f;  // heading (rad)
                ch[12] = v.Speed;                             // airspeed (m/s)
                ch[17] = v.Position.Z;                        // altitude

                _prevVel = vel;
                _qx = qx; _qy = qy; _qz = qz; _qw = qw;
                _hasPrev = true;
            }
            else
            {
                // 下车后清空积分状态, 避免下一帧出现巨大加速度脉冲
                _hasPrev = false;
            }

            // 统一格式: 25 个 "F4" 浮点数, 空格分隔, 与 DCS Lua 一致
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < ch.Length; i++)
            {
                if (i > 0) sb.Append(' ');
                sb.Append(ch[i].ToString("F4", _inv));
            }
            byte[] bytes = Encoding.ASCII.GetBytes(sb.ToString());
            _udp.Send(bytes, bytes.Length, TargetIp, TargetPort);
        }

        private static float Dot(Vector3 a, Vector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// <summary>由前后两帧单位四元数求世界系角速度 (rad/s)。</summary>
        private static Vector3 AngularRate(float px, float py, float pz, float pw,
                                           float cx, float cy, float cz, float cw, float dt)
        {
            // dq = cur * inverse(prev); 单位四元数的逆 = 共轭 (-x,-y,-z,w)
            float ix = -px, iy = -py, iz = -pz, iw = pw;
            float dx = cw * ix + cx * iw + cy * iz - cz * iy;
            float dy = cw * iy - cx * iz + cy * iw + cz * ix;
            float dz = cw * iz + cx * iy - cy * ix + cz * iw;
            float dw = cw * iw - cx * ix - cy * iy - cz * iz;

            if (dw < 0f) { dx = -dx; dy = -dy; dz = -dz; dw = -dw; } // 取最短旋转路径

            float sinHalf = (float)Math.Sqrt(Math.Max(0.0, 1.0 - dw * dw));
            if (sinHalf < 1e-6f) return Vector3.Zero;
            float scale = 2f * (float)Math.Atan2(sinHalf, dw) / (sinHalf * dt);
            return new Vector3(dx * scale, dy * scale, dz * scale);
        }
    }
}
