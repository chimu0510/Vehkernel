#!/usr/bin/env python3
"""
ROS2-UGV Chassis Calibration Tool (ADR-0001, ADR-0002)
Supports:
  1. Wheel Diameter / Linear Scale Calibration (有效輪徑標定)
  2. Track Width / Angular Scale Calibration (有效輪距標定)
  3. Failsafe Deceleration / Ramp Braking Test (最大減速度與斜坡煞車測試)
Uses direct serial communication via ugv_protocol.
"""

import sys
import time
import math
import argparse
from pathlib import Path

# Add protocol directory
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import serial
from ugv_protocol import (
    PROTOCOL_VERSION,
    HeadingMode,
    MessageId,
    HandshakeReq,
    HandshakeResp,
    Heartbeat,
    SetVelocity,
    Telemetry,
    SystemStatusFlags,
    decode_frame_stream,
)


class ChassisCalibrator:
    def __init__(self, port: str, baud: int = 115200, heading: str = "drive_front"):
        self.port = port
        self.baud = baud
        self.heading_mode = (
            HeadingMode.OMNI_FRONT if heading == "omni_front" else HeadingMode.DRIVE_FRONT
        )
        self.ser = None
        self.seq = 0
        self.rx_buf = bytearray()
        self.last_telem = None

    def connect(self):
        print(f"Connecting to C50C on {self.port} at {self.baud} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except (serial.SerialException, OSError) as e:
            print(f"Error: Could not open serial port {self.port}: {e}")
            return False
        time.sleep(0.5)

        # Send Handshake
        print("Sending HandshakeReq...")
        req = HandshakeReq(
            seq=self.next_seq(),
            timestamp_us=0,
            host_proto_version=PROTOCOL_VERSION,
            expected_heading=self.heading_mode,
        )
        self.send_msg(req)

        # Wait for HandshakeResp
        start = time.time()
        while time.time() - start < 3.0:
            msg = self.poll_msg()
            if isinstance(msg, HandshakeResp):
                print(
                    f"Connected! MCU Proto Ver: {msg.mcu_proto_version}, "
                    f"MCU Heading: {msg.mcu_heading}, "
                    f"Heading Matched: {bool(msg.heading_matched)}"
                )
                if not msg.heading_matched:
                    print(
                        f"WARNING: Heading mismatch! Host={self.heading_mode}, MCU={msg.mcu_heading}"
                    )
                return True
            time.sleep(0.01)

        print("Error: Handshake timeout!")
        return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop()
            self.ser.close()

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return self.seq

    def send_msg(self, msg):
        frame = msg.encode_frame()
        self.ser.write(frame)

    def poll_msg(self):
        if not self.ser or not self.ser.is_open:
            return None

        data = self.ser.read(self.ser.in_waiting or 1)
        if not data:
            return None

        self.rx_buf.extend(data)
        while b"\x00" in self.rx_buf:
            idx = self.rx_buf.index(b"\x00")
            frame = bytes(self.rx_buf[:idx])
            del self.rx_buf[: idx + 1]
            if frame:
                try:
                    msg = decode_frame_stream(frame)
                    if isinstance(msg, Telemetry):
                        self.last_telem = msg
                    return msg
                except Exception:
                    pass
        return None

    def send_velocity(self, vx: float, wz: float):
        cmd = SetVelocity(seq=self.next_seq(), timestamp_us=0, target_vx=vx, target_wz=wz)
        self.send_msg(cmd)

    def stop(self):
        for _ in range(5):
            self.send_velocity(0.0, 0.0)
            time.sleep(0.02)

    def calibrate_linear(self, nominal_diameter: float = 0.208, target_distance: float = 1.0):
        print("\n" + "=" * 60)
        print("  1. 有效輪徑標定 (Linear Scale / Wheel Diameter)")
        print("=" * 60)
        print(f"名義輪徑: {nominal_diameter:.4f} m")
        print(f"目標行駛距離: {target_distance:.2f} m")
        print("請確保前方平坦無障礙，並準備好捲尺標記起始線。")
        input("按 [Enter] 開始直行測試...")

        # 讀取初始編碼器
        while self.last_telem is None:
            self.poll_msg()
            time.sleep(0.01)

        start_l = self.last_telem.wheel_pos_l
        start_r = self.last_telem.wheel_pos_r

        # 以 0.25 m/s 前進
        speed = 0.25
        run_time = target_distance / speed
        start_time = time.time()

        print(f"行駛中 (預計 {run_time:.1f} 秒)...")
        while time.time() - start_time < run_time:
            self.send_velocity(speed, 0.0)
            self.poll_msg()
            time.sleep(0.02)

        self.stop()
        print("已平滑煞停。等待 1 秒讀取最終編碼器...")
        time.sleep(1.0)
        for _ in range(20):
            self.poll_msg()
            time.sleep(0.01)

        end_l = self.last_telem.wheel_pos_l
        end_r = self.last_telem.wheel_pos_r

        delta_l = end_l - start_l
        delta_r = end_r - start_r
        avg_counts = (abs(delta_l) + abs(delta_r)) / 2.0

        # 一圈 36000 計數
        circumference = math.pi * nominal_diameter
        measured_dist = (avg_counts / 36000.0) * circumference

        print(f"\n編碼器增量: 左輪={delta_l}, 右輪={delta_r}, 平均={avg_counts:.1f} counts")
        print(f"名義輪徑計算里程計距離: {measured_dist:.4f} m")

        real_dist_str = input(
            f"請輸入用捲尺實際測量的位移距離 (m，直接按 Enter 採用 {measured_dist:.3f}): "
        ).strip()
        real_dist = float(real_dist_str) if real_dist_str else measured_dist

        scale_factor = real_dist / measured_dist if measured_dist > 0 else 1.0
        calibrated_diameter = nominal_diameter * scale_factor

        print("\n--> 標定結果:")
        print(f"    線性修正係數 linear_scale = {scale_factor:.5f}")
        print(f"    校準後有效輪徑 wheel_diameter = {calibrated_diameter:.5f} m")
        return calibrated_diameter, scale_factor

    def calibrate_angular(
        self, nominal_track: float = 0.350, turns: int = 5, nominal_diameter: float = 0.208
    ):
        print("\n" + "=" * 60)
        print("  2. 有效輪距標定 (Angular Scale / Track Width)")
        print("=" * 60)
        print(f"名義輪距: {nominal_track:.4f} m")
        print(f"旋轉圈數: {turns} 圈 ({turns * 360} 度)")
        print("底盤將原地逆時針自轉，請在車體和地面做好對齊標記。")
        input("按 [Enter] 開始旋轉測試...")

        while self.last_telem is None:
            self.poll_msg()
            time.sleep(0.01)

        start_l = self.last_telem.wheel_pos_l
        start_r = self.last_telem.wheel_pos_r

        wz_speed = 0.8  # rad/s
        target_rad = turns * 2.0 * math.pi
        run_time = target_rad / wz_speed
        start_time = time.time()

        print(f"自轉中 (預計 {run_time:.1f} 秒)...")
        while time.time() - start_time < run_time:
            self.send_velocity(0.0, wz_speed)
            self.poll_msg()
            time.sleep(0.02)

        self.stop()
        print("已平滑煞停。等待 1 秒讀取最終編碼器...")
        time.sleep(1.0)
        for _ in range(20):
            self.poll_msg()
            time.sleep(0.01)

        end_l = self.last_telem.wheel_pos_l
        end_r = self.last_telem.wheel_pos_r

        delta_l = end_l - start_l
        delta_r = end_r - start_r

        # 驅動輪行駛弧長
        circ = math.pi * nominal_diameter
        dist_l = (abs(delta_l) / 36000.0) * circ
        dist_r = (abs(delta_r) / 36000.0) * circ
        avg_wheel_arc = (dist_l + dist_r) / 2.0

        measured_rad = avg_wheel_arc / (nominal_track * 0.5)
        measured_deg = math.degrees(measured_rad)
        expected_deg = turns * 360.0

        print(
            f"\n輪子行駛弧長: 左={dist_l:.3f}m, 右={dist_r:.3f}m, 平均={avg_wheel_arc:.3f}m"
        )
        print(f"名義輪距計算轉角: {measured_deg:.2f}° (預期: {expected_deg:.2f}°)")

        actual_turns_str = input(
            f"請觀察車體實際自轉了幾圈 (例如 5.05 或 4.90，直接按 Enter 採用 {turns}): "
        ).strip()
        actual_turns = float(actual_turns_str) if actual_turns_str else float(turns)
        actual_rad = actual_turns * 2.0 * math.pi

        scale_factor = actual_rad / measured_rad if measured_rad > 0 else 1.0
        calibrated_track = nominal_track / scale_factor

        print("\n--> 標定結果:")
        print(f"    旋轉修正係數 angular_scale = {scale_factor:.5f}")
        print(f"    校準後有效輪距 track_width = {calibrated_track:.5f} m")
        return calibrated_track, scale_factor

    def test_failsafe(self, speed: float = 0.8):
        print("\n" + "=" * 60)
        print("  3. 最大減速度與斜坡煞車測試 (Failsafe Deceleration)")
        print("=" * 60)
        print(f"測試目標速度: {speed:.2f} m/s")
        print("請確保前方具備 3 m 以上開闊緩衝區，並注意車體動態。")
        input("按 [Enter] 開始加速並測試失效保護煞車...")

        while self.last_telem is None:
            self.poll_msg()
            time.sleep(0.01)

        print(f"加速至 {speed:.2f} m/s 中...")
        accel_duration = 2.0
        start_time = time.time()
        while time.time() - start_time < accel_duration:
            self.send_velocity(speed, 0.0)
            self.poll_msg()
            time.sleep(0.02)

        peak_speed = self.last_telem.measured_vx if self.last_telem else speed
        print(f"達到目標速度 (遙測 Vx: {peak_speed:.3f} m/s)。")
        if abs(peak_speed) < abs(speed) * 0.6:
            print(
                f"警告: 遙測車速 ({peak_speed:.3f} m/s) 明顯低於目標速度 ({speed:.2f} m/s)！\n"
                f"      可能原因：馬達轉向相反導致原地打轉、編碼器極性反接、或輪胎打滑。"
            )

        print("模擬通訊斷開 (停止下發心跳/速度指令)，等待下位機觸發失效保護斜坡煞車...")

        disconn_time = time.time()
        samples = []
        failsafe_detected = False
        fs_trigger_time = None
        stopped = False
        stop_time = None

        # 最多監聽 5 秒
        zero_count = 0
        while time.time() - disconn_time < 5.0:
            msg = self.poll_msg()
            now = time.time()
            if isinstance(msg, Telemetry):
                t_rel = now - disconn_time
                vx = msg.measured_vx
                fs_active = bool(msg.status_flags & SystemStatusFlags.FLAG_FAILSAFE_ACTIVE)
                if fs_active:
                    failsafe_detected = True
                    if fs_trigger_time is None:
                        fs_trigger_time = now
                samples.append((t_rel, vx, fs_active))

                # 判定停止: 必須已超過下位機通訊超時門檻 (> 0.5s 或已進入 failsafe)，且連續數個週期速度穩定接近 0
                if (failsafe_detected or t_rel > 0.5) and abs(vx) < 0.03:
                    zero_count += 1
                    if zero_count >= 5 and stop_time is None:
                        stop_time = now
                        stopped = True
                        break
                else:
                    zero_count = 0
            time.sleep(0.01)

        print("\n" + "-" * 60)
        print("測試結果與減速數據分析:")
        print(f"  初始車速: {peak_speed:.3f} m/s")
        print(
            f"  是否成功觸發失效保護 (FLAG_FAILSAFE_ACTIVE): {'是' if failsafe_detected else '否 (或逾時門檻尚未達到)'}"
        )

        if stop_time is not None:
            t_total = stop_time - disconn_time
            if fs_trigger_time is not None:
                t_latency = fs_trigger_time - disconn_time
                t_brake = stop_time - fs_trigger_time
                a_brake = abs(peak_speed) / t_brake if t_brake > 0 else 0.0
                print(f"  通訊中斷反應延遲 t_detect: {t_latency:.3f} s (下位機超時門檻 0.25 s)")
                print(f"  實際斜坡煞停耗時 t_brake: {t_brake:.3f} s")
                print(f"  實測斜坡減速度 a_brake: {a_brake:.3f} m/s^2")
            else:
                a_brake = abs(peak_speed) / t_total if t_total > 0 else 0.0
                print(f"  停止總耗時 t_stop: {t_total:.3f} s (含通訊超時判定)")
                print(f"  等效平均減速度 a_brake: {a_brake:.3f} m/s^2")

            if abs(peak_speed) < 0.3:
                print("  安全評估: INVALID (初始車速過低，無法有效評估斜坡減速行為)")
            elif a_brake <= 1.5:
                print("  安全評估: PASS (a_brake <= 1.5 m/s^2，符合平滑斜坡煞車規範)")
            else:
                print("  安全評估: WARNING (a_brake > 1.5 m/s^2，減速過猛可能有衝擊風險)")
        else:
            print("  停止檢測: 5 秒內未完全停止或遙測中斷。")

        if samples:
            print("\n  減速斜坡曲線採樣 (時間, Vx, Failsafe):")
            step = max(1, len(samples) // 10)
            for s in samples[::step]:
                print(f"    t={s[0]:.2f}s | Vx={s[1]:.3f} m/s | Failsafe={s[2]}")
            if samples[-1] != samples[::step][-1]:
                s = samples[-1]
                print(f"    t={s[0]:.2f}s | Vx={s[1]:.3f} m/s | Failsafe={s[2]}")

        print("-" * 60)


def main():
    parser = argparse.ArgumentParser(description="ROS2-UGV Chassis Calibration Tool")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument(
        "--heading",
        default="drive_front",
        choices=["drive_front", "omni_front"],
        help="Heading mode (default: drive_front)",
    )
    parser.add_argument(
        "--mode",
        default="all",
        choices=["all", "linear", "angular", "failsafe"],
        help="Calibration mode: linear, angular, failsafe, or all (default: all)",
    )
    parser.add_argument(
        "--dist",
        type=float,
        default=2.0,
        help="Target travel distance for linear calibration in meters (default: 2.0)",
    )
    parser.add_argument(
        "--turns",
        type=float,
        default=5.0,
        help="Number of in-place rotation turns for angular calibration (default: 5.0)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.8,
        help="Target speed for failsafe deceleration test in m/s (default: 0.8)",
    )
    args = parser.parse_args()

    cal = ChassisCalibrator(port=args.port, baud=args.baud, heading=args.heading)
    if not cal.connect():
        sys.exit(1)

    try:
        if args.mode == "linear":
            cal.calibrate_linear(target_distance=args.dist)
        elif args.mode == "angular":
            cal.calibrate_angular(turns=args.turns)
        elif args.mode == "failsafe":
            cal.test_failsafe(speed=args.speed)
        elif args.mode == "all":
            calibrated_d, lin_scale = cal.calibrate_linear(target_distance=args.dist)
            calibrated_t, ang_scale = cal.calibrate_angular(
                nominal_diameter=calibrated_d, turns=args.turns
            )

            print("\n" + "=" * 60)
            print("  標定總結報告 (可直接套用於 ugv_base_params.yaml)")
            print("=" * 60)
            print(f"linear_scale: {lin_scale:.5f}")
            print(f"angular_scale: {ang_scale:.5f}")
            print(f"effective_wheel_diameter: {calibrated_d:.5f}  # m")
            print(f"effective_track_width: {calibrated_t:.5f}     # m")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\n使用者中斷，停止底盤。")
    finally:
        cal.close()


if __name__ == "__main__":
    main()
