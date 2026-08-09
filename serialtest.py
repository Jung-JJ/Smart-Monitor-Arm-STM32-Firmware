import serial
import struct
import threading
import time

PORT = "COM9"
BAUD = 115200

START = 0xAA
END = 0x55

# =========================
# Python -> STM32
# =========================

MSG_HEARTBEAT = 0x01
MSG_SET_TARGET = 0x10
MSG_SET_HOME = 0x11
MSG_MOVE_HOME = 0x12
MSG_JOG = 0x13

# =========================
# STM32 -> Python
# =========================

MSG_ACK = 0x80
MSG_STATUS = 0x81
MSG_MOVE_DONE = 0x82
MSG_ERROR = 0x83
MSG_CURRENT_ANGLE = 0x84
MSG_CURRENT_COMMAND_ANGLES = 0x85
MSG_READY = 0x86

# =========================
# JOG AXIS
# =========================

AXIS_THETA1 = 1
AXIS_THETA2 = 2
AXIS_THETA3 = 3

alive_counter = 0

write_lock = threading.Lock()


# ============================================================
# CHECKSUM
# ============================================================

def checksum(msg_id, data):
    cs = msg_id ^ len(data)

    for byte in data:
        cs ^= byte

    return cs & 0xFF


# ============================================================
# FRAME BUILD
# ============================================================

def build_frame(msg_id, data):
    frame = bytearray()

    frame.append(START)
    frame.append(msg_id)
    frame.append(len(data))

    frame.extend(data)

    frame.append(
        checksum(msg_id, data)
    )

    frame.append(END)

    return frame


# ============================================================
# SAFE UART WRITE
# ============================================================

def safe_write(ser, frame):
    with write_lock:
        ser.write(frame)
        ser.flush()


# ============================================================
# HEARTBEAT THREAD
# ============================================================

def heartbeat_thread(ser):
    global alive_counter

    last_time = time.monotonic()

    while ser.is_open:

        # Heartbeat 실제 실행 간격 측정
        now = time.monotonic()
        interval = now - last_time
        last_time = now

        print(
            f"\n[HB TX] interval={interval:.3f}s"
        )

        # Alive counter 증가
        alive_counter = (
            alive_counter + 1
        ) & 0xFF

        data = bytes(
            [alive_counter]
        )

        frame = build_frame(
            MSG_HEARTBEAT,
            data
        )

        try:
            safe_write(
                ser,
                frame
            )

        except serial.SerialException:
            break

        time.sleep(1.0)


# ============================================================
# SET_TARGET
# ============================================================

def send_set_target(
    ser,
    theta1_deg,
    theta2_deg,
    theta3_deg
):

    theta1_x10 = int(
        round(theta1_deg * 10.0)
    )

    theta2_x10 = int(
        round(theta2_deg * 10.0)
    )

    theta3_x10 = int(
        round(theta3_deg * 10.0)
    )

    data = struct.pack(
        ">hhh",
        theta1_x10,
        theta2_x10,
        theta3_x10
    )

    frame = build_frame(
        MSG_SET_TARGET,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print(
        f"SET_TARGET TX : "
        f"theta1={theta1_deg:+.1f} deg, "
        f"theta2={theta2_deg:+.1f} deg, "
        f"theta3={theta3_deg:+.1f} deg"
    )

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# JOG
# ============================================================

def send_jog(
    ser,
    axis,
    delta_deg
):

    delta_x10 = int(
        round(delta_deg * 10.0)
    )

    data = bytearray()

    data.append(axis)

    data.extend(
        struct.pack(
            ">h",
            delta_x10
        )
    )

    frame = build_frame(
        MSG_JOG,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print(
        f"JOG TX : "
        f"axis={axis}, "
        f"delta={delta_deg:+.1f} deg"
    )

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# SET_HOME
# ============================================================

def send_set_home(ser):

    data = b""

    frame = build_frame(
        MSG_SET_HOME,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print("SET_HOME TX")

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# MOVE_HOME
# ============================================================

def send_move_home(ser):

    data = b""

    frame = build_frame(
        MSG_MOVE_HOME,
        data
    )

    safe_write(
        ser,
        frame
    )

    print()
    print("MOVE_HOME TX")

    print(
        "TX :",
        frame.hex(" ").upper()
    )


# ============================================================
# RX THREAD
# ============================================================

def receive_thread(ser):

    while ser.is_open:

        start = ser.read(1)

        if start != bytes([START]):
            continue

        header = ser.read(2)

        if len(header) != 2:
            continue

        msg_id = header[0]
        length = header[1]

        data = ser.read(length)

        if len(data) != length:
            print(
                "DATA LENGTH ERROR"
            )
            continue

        rx_checksum = ser.read(1)

        if len(rx_checksum) != 1:
            print(
                "CHECKSUM BYTE ERROR"
            )
            continue

        end = ser.read(1)

        if end != bytes([END]):
            print(
                "END BYTE ERROR"
            )
            continue

        calc_checksum = checksum(
            msg_id,
            data
        )

        if calc_checksum != rx_checksum[0]:

            print(
                f"CHECKSUM ERROR : "
                f"RX=0x{rx_checksum[0]:02X}, "
                f"CALC=0x{calc_checksum:02X}"
            )

            continue

        # =============================================
        # ACK
        # =============================================

        if msg_id == MSG_ACK:

            if len(data) != 1:
                print(
                    "ACK LENGTH ERROR"
                )
                continue

            ack_id = data[0]

            if ack_id == MSG_HEARTBEAT:

                print(
                    "ACK : HEARTBEAT (0x01)"
                )

            elif ack_id == MSG_SET_TARGET:

                print(
                    "ACK : SET_TARGET (0x10)"
                )

            elif ack_id == MSG_SET_HOME:

                print(
                    "ACK : SET_HOME (0x11)"
                )

            elif ack_id == MSG_MOVE_HOME:

                print(
                    "ACK : MOVE_HOME (0x12)"
                )

            elif ack_id == MSG_JOG:

                print(
                    "ACK : JOG (0x13)"
                )

            else:

                print(
                    f"ACK : "
                    f"0x{ack_id:02X}"
                )

        # =============================================
        # MOVE DONE
        # =============================================

        elif msg_id == MSG_MOVE_DONE:

            print(
                "MOVE DONE"
            )

        # =============================================
        # ERROR
        # =============================================

        elif msg_id == MSG_ERROR:
            if len(data) == 1:
                error_code = data[0]

                error_names = {
                    0x00: "NONE",
                    0x01: "INVALID_TARGET",
                    0x02: "INVALID_AXIS",
                    0x03: "ANGLE_LIMIT",
                    0x04: "STEPPER_BUSY",
                    0x05: "STEPPER",
                    0x06: "SERVO2",
                    0x07: "SERVO3",
                    0x08: "HOME_NOT_SET",
                    0x09: "INIT",
                    0x0A: "QUEUE",
                    0x0B: "INVALID_COMMAND",
                }

                print(
                    f"MOTOR ERROR : "
                    f"{error_names.get(error_code, f'UNKNOWN(0x{error_code:02X})')}"
                )
            else:
                print(
                    f"MOTOR ERROR : INVALID DATA LENGTH ({len(data)})"
                )
        # =============================================
        # AS5600 ACTUAL THETA1
        # =============================================

        elif msg_id == MSG_CURRENT_ANGLE:

            print(
                f"0x84 RX RAW : "
                f"len={len(data)}, "
                f"data={data.hex(' ').upper()}"
            )

            if len(data) == 2:

                angle_x10 = struct.unpack(
                    ">h",
                    data
                )[0]

                angle_deg = (
                    angle_x10 / 10.0
                )

                print(
                    f"ENCODER ANGLE : "
                    f"{angle_deg:+.1f} deg"
                )

            else:

                print(
                    f"0x84 LENGTH ERROR : "
                    f"{len(data)}"
                )

        # =============================================
        # CURRENT COMMAND / ESTIMATED ANGLES
        # =============================================

        elif msg_id == MSG_CURRENT_COMMAND_ANGLES:

            print(
                f"0x85 RX RAW : "
                f"len={len(data)}, "
                f"data={data.hex(' ').upper()}"
            )

            if len(data) == 6:

                (
                    theta1_x10,
                    theta2_x10,
                    theta3_x10
                ) = struct.unpack(
                    ">hhh",
                    data
                )

                theta1 = (
                    theta1_x10 / 10.0
                )

                theta2 = (
                    theta2_x10 / 10.0
                )

                theta3 = (
                    theta3_x10 / 10.0
                )

                print(
                    f"CURRENT COMMAND ANGLES : "
                    f"theta1={theta1:+.1f} deg, "
                    f"theta2={theta2:+.1f} deg, "
                    f"theta3={theta3:+.1f} deg"
                )

            else:

                print(
                    f"0x85 LENGTH ERROR : "
                    f"{len(data)}"
                )

        # =============================================
        # STATUS
        # =============================================

        elif msg_id == MSG_STATUS:

            print(
                "STATUS :",
                data.hex(" ").upper()
            )

        # =============================================
        # UNKNOWN
        # =============================================
        elif msg_id == MSG_READY:
            print("STM32 READY")

        else:

            print(
                f"UNKNOWN RX : "
                f"MSG=0x{msg_id:02X}, "
                f"LEN={length}, "
                f"DATA={data.hex(' ').upper()}"
            )


# ============================================================
# MENU
# ============================================================

def print_menu():

    print()
    print(
        "========== MOTOR TEST =========="
    )

    print(
        "t  : SET_TARGET"
    )

    print()

    print(
        "1+ : theta1 JOG +1 deg"
    )

    print(
        "1- : theta1 JOG -1 deg"
    )

    print(
        "2+ : theta2 JOG +1 deg"
    )

    print(
        "2- : theta2 JOG -1 deg"
    )

    print(
        "3+ : theta3 JOG +1 deg"
    )

    print(
        "3- : theta3 JOG -1 deg"
    )

    print()

    print(
        "h  : SET_HOME"
    )

    print(
        "m  : MOVE_HOME"
    )

    print()

    print(
        "q  : quit"
    )

    print(
        "================================"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    try:

        with serial.Serial(
            PORT,
            BAUD,
            timeout=0.1,
            write_timeout=1.0
        ) as ser:

            print(
                f"Serial Open : "
                f"{PORT} / {BAUD}"
            )

            time.sleep(2.0)

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # RX thread
            threading.Thread(
                target=receive_thread,
                args=(ser,),
                daemon=True
            ).start()

            # Heartbeat thread
            threading.Thread(
                target=heartbeat_thread,
                args=(ser,),
                daemon=True
            ).start()

            print_menu()

            while True:

                command = input(
                    "\n> "
                ).strip()

                # =====================================
                # SET_TARGET
                # =====================================

                if command == "t":

                    theta1 = float(
                        input(
                            "theta1 target : "
                        )
                    )

                    theta2 = float(
                        input(
                            "theta2 target : "
                        )
                    )

                    theta3 = float(
                        input(
                            "theta3 target : "
                        )
                    )

                    send_set_target(
                        ser,
                        theta1,
                        theta2,
                        theta3
                    )

                # =====================================
                # THETA1 JOG
                # =====================================

                elif command == "1+":

                    send_jog(
                        ser,
                        AXIS_THETA1,
                        +1.0
                    )

                elif command == "1-":

                    send_jog(
                        ser,
                        AXIS_THETA1,
                        -1.0
                    )

                # =====================================
                # THETA2 JOG
                # =====================================

                elif command == "2+":

                    send_jog(
                        ser,
                        AXIS_THETA2,
                        +1.0
                    )

                elif command == "2-":

                    send_jog(
                        ser,
                        AXIS_THETA2,
                        -1.0
                    )

                # =====================================
                # THETA3 JOG
                # =====================================

                elif command == "3+":

                    send_jog(
                        ser,
                        AXIS_THETA3,
                        +1.0
                    )

                elif command == "3-":

                    send_jog(
                        ser,
                        AXIS_THETA3,
                        -1.0
                    )

                # =====================================
                # SET_HOME
                # =====================================

                elif command == "h":

                    send_set_home(
                        ser
                    )

                # =====================================
                # MOVE_HOME
                # =====================================

                elif command == "m":

                    send_move_home(
                        ser
                    )

                # =====================================
                # QUIT
                # =====================================

                elif command == "q":

                    print(
                        "Exit"
                    )

                    break

                else:

                    print(
                        "Unknown command"
                    )

                    print_menu()

    except serial.SerialException as exc:

        print(
            f"Serial Error : {exc}"
        )

    except ValueError:

        print(
            "숫자를 입력해야 합니다."
        )

    except KeyboardInterrupt:

        print(
            "\nStopped"
        )


if __name__ == "__main__":
    main()