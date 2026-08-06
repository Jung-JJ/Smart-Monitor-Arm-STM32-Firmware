import serial
import struct
import threading
import time

PORT = "COM9"
BAUD = 115200

START = 0xAA
END = 0x55

MSG_HEARTBEAT = 0x01
MSG_SET_TARGET = 0x10

MSG_ACK = 0x80
MSG_STATUS = 0x81
MSG_MOVE_DONE = 0x82
MSG_ERROR = 0x83
MSG_CURRENT_ANGLE = 0x84

alive_counter = 0
serial_write_lock = threading.Lock()


def checksum(msg_id, data):
    cs = msg_id ^ len(data)

    for b in data:
        cs ^= b

    return cs & 0xFF


def build_frame(msg_id, data):
    frame = bytearray()

    frame.append(START)
    frame.append(msg_id)
    frame.append(len(data))
    frame.extend(data)
    frame.append(checksum(msg_id, data))
    frame.append(END)

    return frame


def safe_write(ser, frame):
    with serial_write_lock:
        ser.write(frame)
        ser.flush()


def heartbeat_thread(ser):
    global alive_counter

    while ser.is_open:
        alive_counter = (alive_counter + 1) & 0xFF

        frame = build_frame(
            MSG_HEARTBEAT,
            bytes([alive_counter])
        )

        safe_write(ser, frame)
        time.sleep(1.0)


def send_target(ser, theta1, theta2, theta3):
    data = struct.pack(
        ">hhh",
        theta1,
        theta2,
        theta3
    )

    frame = build_frame(
        MSG_SET_TARGET,
        data
    )

    safe_write(ser, frame)

    print("SET_TARGET TX")
    print("TX:", frame.hex(" ").upper())


def receive_loop(ser):
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
            continue

        cs = ser.read(1)
        end = ser.read(1)

        if len(cs) != 1:
            continue

        if end != bytes([END]):
            continue

        if checksum(msg_id, data) != cs[0]:
            print("Checksum Error")
            continue

        if msg_id == MSG_ACK:
            if len(data) == 1:
                print(f"ACK : 0x{data[0]:02X}")

        elif msg_id == MSG_CURRENT_ANGLE:
            if len(data) == 2:
                angle_x10 = struct.unpack(">h", data)[0]
                print(f"ANGLE : {angle_x10 / 10.0:.1f} deg")

        elif msg_id == MSG_STATUS:
            print("STATUS:", data.hex(" ").upper())

        elif msg_id == MSG_MOVE_DONE:
            print("MOVE DONE:", data.hex(" ").upper())

        elif msg_id == MSG_ERROR:
            print("ERROR:", data.hex(" ").upper())

        else:
            print(
                f"UNKNOWN 0x{msg_id:02X}:",
                data.hex(" ").upper()
            )


def main():
    try:
        with serial.Serial(
            PORT,
            BAUD,
            timeout=0.1
        ) as ser:

            time.sleep(2.0)

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # 먼저 SET_TARGET 전송
            send_target(
                ser,
                10,
                0,
                0
            )

            # SET_TARGET과 첫 heartbeat write가 겹치지 않게 약간 지연
            time.sleep(0.2)

            threading.Thread(
                target=heartbeat_thread,
                args=(ser,),
                daemon=True
            ).start()

            receive_loop(ser)

    except serial.SerialException as exc:
        print(f"Serial error: {exc}")

    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()