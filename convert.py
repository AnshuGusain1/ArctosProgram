import os
import re
from typing import List

# Parameters Section
# -------------------

# Gear ratios for each motor (joint angle to motor angle)
gear_ratios: List[float] = [
    13.5,    # X (J1) - Belt reduction
    150,     # Y (J2) - Cycloidal
    150,     # Z (J3) - Cycloidal
    48,      # A (J4) - Compound Planetary
    67.82,   # B (J5) - Compound Planetary
    67.82,   # C (J6) - Compound Planetary
]

# Direction inversion for each motor
invert_direction: List[bool] = [
    True,   # X (J1)
    True,   # Y (J2)
    False,  # Z (J3)
    False,  # A (J4)
    False,  # B (J5)
    False,  # C (J6)
]

# Motor CAN ID offset: axis_id (1-6) + offset = CAN ID on bus
# axis 1 -> 0x07, axis 2 -> 0x08, ... axis 6 -> 0x0C
MOTOR_CAN_ID_OFFSET = 6

# End-effector
END_EFFECTOR_CAN_ID = 0x0D
CMD_SET_PWM         = 0x03

# Last known motor positions in degrees, used for relative position calculation
last_motor_positions: List[float] = [0.0] * 6

# -------------------


def joint_to_motor_positions(joint_angles: List[float]) -> List[float]:
    """
    Converts joint angles (degrees) to motor positions (degrees),
    applying gear ratios, direction inversion, and differential coupling for J5/J6.

    Args:
        joint_angles: List of 6 joint angles [q1..q6] in degrees.

    Returns:
        List of 6 motor positions in degrees.
    """
    q1, q2, q3, q4, q5, q6 = joint_angles

    motor_positions = [
        q1 * gear_ratios[0],
        q2 * gear_ratios[1],
        q3 * gear_ratios[2],
        q4 * gear_ratios[3],
        (q5 + q6) * gear_ratios[4],  # Differential coupling
        (q5 - q6) * gear_ratios[5],  # Differential coupling
    ]

    for i in range(6):
        if invert_direction[i]:
            motor_positions[i] = -motor_positions[i]

    return motor_positions


def calculate_crc(data: List[int]) -> int:
    """
    Calculates CRC as the sum of data bytes mod 256.

    IMPORTANT: pass only the 7 DATA bytes — do NOT include the CAN ID byte.
    The CAN ID is the arbitration ID (separate field on the bus) and is not
    present in buf[] on the Arduino receiver side.

    Args:
        data: List of 7 ints representing the data bytes of the CAN frame.

    Returns:
        CRC byte (0x00-0xFF).
    """
    return sum(data) & 0xFF


def convert_to_can_message(axis_id: int, speed: int, motor_position: float) -> str:
    """
    Converts a pre-computed motor position into a CAN message string.
    Gear ratio, inversion, and differential coupling must already be applied
    (via joint_to_motor_positions) before calling this.

    Message layout (8 bytes total on the wire):
        [arbitration_id] | Byte0  Byte1  Byte2  Byte3  Byte4  Byte5  Byte6  Byte7
        [  axis_id+6   ] | 0xF5  speed_hi speed_lo  0x02  pos[2] pos[1] pos[0]  CRC

        CRC = sum(Byte0..Byte6) & 0xFF   <-- data bytes only, no CAN ID

    Args:
        axis_id:        Motor axis index (1-6).
        speed:          Feed speed.
        motor_position: Target motor position in degrees (pre-converted from joint angle).

    Returns:
        16-char hex string: 2-char CAN ID + 14-char data + 2-char CRC.
        parse_can_message() in send.py strips the first 2 chars as arbitration_id.
    """
    can_id    = format(axis_id + MOTOR_CAN_ID_OFFSET, "02X")
    speed_hex = format(speed, "04X")

    rel_position     = int((motor_position - last_motor_positions[axis_id - 1]) * 100)
    rel_position_hex = format(rel_position & 0xFFFFFF, "06X")  # signed 24-bit two's complement

    last_motor_positions[axis_id - 1] = motor_position

    data_hex  = "F5" + speed_hex + "02" + rel_position_hex          # 7 data bytes as hex
    data_ints = [int(data_hex[i:i+2], 16) for i in range(0, len(data_hex), 2)]
    crc       = calculate_crc(data_ints)

    return can_id + data_hex + format(crc, "02X")


def convert_to_end_effector_message(brightness: int, speed: int = 0) -> str:
    """
    Converts an LED brightness value into a CAN message for the end-effector (ID 0x0D).

    Message layout (8 bytes total on the wire):
        [arbitration_id] | Byte0  Byte1  Byte2  Byte3  Byte4  Byte5  Byte6  Byte7
        [    0x0D      ] | 0xF5  spd_hi spd_lo  0x03   0x00  brite  0x00   CRC

        CRC = sum(Byte0..Byte6) & 0xFF   <-- data bytes only, no CAN ID

    Args:
        brightness: LED PWM value, 0-255.
        speed:      Unused for now, kept for protocol consistency.

    Returns:
        16-char hex string: 2-char CAN ID + 14-char data + 2-char CRC.
    """
    can_id         = format(END_EFFECTOR_CAN_ID, "02X")
    speed_hex      = format(speed, "04X")
    command        = format(CMD_SET_PWM, "02X")
    brightness_hex = format(brightness & 0xFF, "02X")

    data_hex  = "F5" + speed_hex + command + "00" + brightness_hex + "00"  # 7 data bytes
    data_ints = [int(data_hex[i:i+2], 16) for i in range(0, len(data_hex), 2)]
    crc       = calculate_crc(data_ints)

    return can_id + data_hex + format(crc, "02X")


def process_tap_files() -> None:
    """
    Processes all .tap files in the script directory, converting them into
    .txt files containing CAN messages (7 lines per G90 block: 6 motors + end-effector).

    G-code format expected:
        F<speed>                       -> sets feed speed for subsequent moves
        G90 X Y Z A B C E<brightness> -> absolute move + end-effector brightness (0-255)

    Each output line is a 16-char hex string:
        chars 0-1:   CAN arbitration ID
        chars 2-15:  7 data bytes
        chars 14-15: CRC (last byte, sum of data bytes 0-6 mod 256)
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))

    for filename in os.listdir(script_dir):
        if not filename.endswith(".tap"):
            continue

        input_path  = os.path.join(script_dir, filename)
        output_path = os.path.join(script_dir, os.path.splitext(filename)[0] + ".txt")

        print(f"\nProcessing: {filename} -> {os.path.basename(output_path)}")

        with open(input_path, "r") as input_file, open(output_path, "w") as output_file:
            speed = 0

            for line_num, raw_line in enumerate(input_file, start=1):
                line = raw_line.strip()

                if not line or line.startswith(";"):
                    continue

                # --- Speed update (standalone F line or inline on G90) ---
                speed_match = re.search(r"F(\d+)", line)
                if speed_match:
                    speed = int(speed_match.group(1))
                    if not line.startswith("G90"):
                        continue

                if not line.startswith("G90"):
                    continue

                # --- Parse joint angles from X Y Z A B C fields ---
                axis_values = re.findall(r"[XYZABC]([-+]?\d*\.?\d+)", line)
                e_match     = re.search(r"E(\d+)", line)

                if len(axis_values) != 6:
                    print(f"  WARNING line {line_num}: expected 6 axis values, "
                          f"got {len(axis_values)} -- skipping: {line}")
                    continue

                joint_angles    = [float(v) for v in axis_values]
                motor_positions = joint_to_motor_positions(joint_angles)

                # --- Motor messages (axes 1-6, CAN IDs 0x07-0x0C) ---
                for axis_id, motor_pos in enumerate(motor_positions, start=1):
                    full_msg = convert_to_can_message(axis_id, speed, motor_pos)
                    output_file.write(full_msg + "\n")
                    print(f"  Motor {axis_id} (ID 0x{axis_id + MOTOR_CAN_ID_OFFSET:02X}): {full_msg}"
                          f"  joint={joint_angles[axis_id-1]:.2f}  motor={motor_pos:.2f}deg")

                # --- End-effector message (CAN ID 0x0D) ---
                brightness = int(float(e_match.group(1))) if e_match else 0
                brightness = max(0, min(255, brightness))

                ee_msg = convert_to_end_effector_message(brightness)
                output_file.write(ee_msg + "\n")
                print(f"  End-effector  (ID 0x{END_EFFECTOR_CAN_ID:02X}):  {ee_msg}"
                      f"  brightness={brightness}")

        print(f"Done: {output_path}")


if __name__ == "__main__":
    process_tap_files()