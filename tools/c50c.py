#!/usr/bin/env python3
"""C50C 控制板的燒錄、重設與串口監看。

  tools/c50c.py flash 01_board_info              # 經 Type-C 串口 1 燒錄（預設）
  tools/c50c.py flash 01_board_info --swd        # 經 SWD（ST-Link）燒錄
  tools/c50c.py monitor                          # 開啟主控台（115200）
  tools/c50c.py reset                            # 重設並執行使用者程式
  tools/c50c.py ports                            # 列出 CH9102F 串口

燒錄目標可以是範例名稱（取 examples/build/<名稱>.bin）或 .bin/.hex/.elf 路徑。

串口自動下載電路（C50C-V4.1V4.2 原理圖第 3 頁）：
  - RTS 經 PNP（Q6）驅動 BOOT0：RTS 腳為低 → BOOT0 為高。
  - DTR 經 NPN（Q5，射極接 RTS）拉 NRST：DTR 腳為高且 RTS 腳為低 → NRST 被拉低。
pyserial 的 rts/dtr = True 代表訊號有效，也就是腳位為低電位。
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

REPO = Path(__file__).resolve().parent.parent
BUILD_DIR = REPO / "examples" / "build"
FLASH_BASE = "0x08000000"
BAUD = 115200
WCH_VID = 0x1A86


def find_programmer():
    exe = os.environ.get("STM32_PROGRAMMER_CLI") or shutil.which("STM32_Programmer_CLI")
    if exe:
        return exe
    pattern = str(Path.home() / "*" / "STMicroelectronics" / "STM32Cube" /
                  "STM32CubeProgrammer" / "bin" / "STM32_Programmer_CLI")
    found = glob.glob(pattern) + glob.glob(
        "/opt/st/*/STM32CubeProgrammer/bin/STM32_Programmer_CLI")
    if found:
        return found[0]
    sys.exit("STM32_Programmer_CLI not found; add it to PATH or set STM32_PROGRAMMER_CLI")


def wch_ports():
    return [p for p in list_ports.comports() if p.vid == WCH_VID]


def resolve_port(port):
    if port:
        return port
    ports = wch_ports()
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        sys.exit("no CH9102F serial port found; is the Type-C cable connected?")
    listing = "\n".join(f"  {p.device}  {p.description}  serial={p.serial_number}" for p in ports)
    sys.exit("several CH9102F ports found, pick the one on USART1 with --port:\n" + listing)


def resolve_image(target):
    path = Path(target)
    if path.suffix in (".bin", ".hex", ".elf") and path.exists():
        return path
    image = BUILD_DIR / f"{target}.bin"
    if not image.exists():
        sys.exit(f"{image} not found; build first: cmake --build examples/build")
    return image


def open_quiet(port):
    """開啟串口而不觸發重設：RTS、DTR 都設為無效（腳位為高）。"""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.rts = False
    ser.dtr = False
    ser.open()
    return ser


def enter_bootloader(port):
    with open_quiet(port) as ser:
        ser.dtr = False
        ser.rts = True      # BOOT0 高，NRST 被拉低
        time.sleep(0.1)
        ser.dtr = True      # 放開 NRST，BOOT0 仍為高 → 進入內建 bootloader
        time.sleep(0.1)
        ser.rts = False     # bootloader 已經執行，BOOT0 可以放回低


def pulse_reset(ser):
    ser.dtr = False
    ser.rts = True      # 拉低 NRST
    time.sleep(0.1)
    ser.rts = False     # 同時放開 NRST 與 BOOT0；NRST 的 RC 讓 BOOT0 先回到低


def reset_to_app(port):
    with open_quiet(port) as ser:
        pulse_reset(ser)
        time.sleep(0.05)


def cmd_flash(args):
    image = resolve_image(args.target)
    cli = find_programmer()
    if args.swd:
        cmd = [cli, "-c", "port=SWD", "mode=UR", "-w", str(image)]
        if image.suffix == ".bin":
            cmd.append(FLASH_BASE)
        subprocess.run(cmd + ["-v", "-rst"], check=True)
        return
    port = resolve_port(args.port)
    enter_bootloader(port)
    cmd = [cli, "-c", f"port={port}", f"br={BAUD}", "-w", str(image)]
    if image.suffix == ".bin":
        cmd.append(FLASH_BASE)
    subprocess.run(cmd + ["-v"], check=True)
    reset_to_app(port)
    print(f"flashed {image.name} via {port}; board reset")


def cmd_reset(args):
    reset_to_app(resolve_port(args.port))


def cmd_monitor(args):
    from serial.tools.miniterm import Miniterm

    port = resolve_port(args.port)
    ser = open_quiet(port)
    term = Miniterm(ser, echo=False, eol="crlf", filters=["default"])
    term.exit_character = chr(0x1D)  # Ctrl+]
    term.menu_character = chr(0x14)  # Ctrl+T
    term.set_rx_encoding("UTF-8")
    term.set_tx_encoding("UTF-8")
    print(f"--- {port} {BAUD} 8N1 | quit: Ctrl+] ---", file=sys.stderr)
    term.start()
    # 終端機已經在讀取後才重設，板子開機後立刻印出的內容就不會遺失。
    if args.reset:
        pulse_reset(ser)
    try:
        term.join(True)
    except KeyboardInterrupt:
        pass
    term.join()
    term.close()


def cmd_ports(_args):
    for p in wch_ports():
        print(f"{p.device}  {p.description}  serial={p.serial_number}  location={p.location}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    flash = sub.add_parser("flash", help="flash an example or image")
    flash.add_argument("target", help="example name (e.g. 01_board_info) or .bin/.hex/.elf path")
    flash.add_argument("--swd", action="store_true", help="use SWD (ST-Link) instead of serial")
    flash.add_argument("--port", help="serial port on USART1 (default: the only CH9102F)")
    flash.set_defaults(func=cmd_flash)

    reset = sub.add_parser("reset", help="reset the board into the user program")
    reset.add_argument("--port")
    reset.set_defaults(func=cmd_reset)

    monitor = sub.add_parser("monitor", help="open the console on USART1")
    monitor.add_argument("--port")
    monitor.add_argument("--reset", action="store_true", help="reset the board first")
    monitor.set_defaults(func=cmd_monitor)

    ports = sub.add_parser("ports", help="list CH9102F serial ports")
    ports.set_defaults(func=cmd_ports)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
