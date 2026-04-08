import os
import sys
import time
from typing import List

import can

# CAN interface config
CAN_INTERFACE = "slcan"
CAN_CHANNEL   = "/dev/tty.usbmodem2095387B58421"
CAN_BITRATE   = 500000

# Lines per packet: 6 motors + 1 end-effector
PACKET_SIZE = 7

# End-effector CAN ID — used to skip speed adjustment for non-motor messages
END_EFFECTOR_CAN_ID = 0x0D


def parse_can_message(line: str) -> can.Message:
    """
    Parses a single hex line from the .txt file into a can.Message.

    Line format (16 hex chars, no spaces):
        chars 0-1:   CAN arbitration ID
        chars 2-13:  6 data bytes
        chars 14-15: CRC byte

    Args:
        line: 16-char hex string, e.g. "07F503E80200000081"

    Returns:
        can.Message with arbitration_id and data populated.
    """
    line = line.strip()
    arbitration_id = int(line[:2], 16)
    data           = [int(line[i:i+2], 16) for i in range(2, len(line), 2)]
    return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)


def adjust_speeds_within_packet(messages: List[can.Message]) -> None:
    """
    Adjusts motor speeds within a packet to a common reference (average),
    then recomputes CRC for any modified messages.

    Skips the end-effector message (ID 0x0D) since it has no speed field.

    CRC is sum of data bytes 0-6 mod 256 (does NOT include arbitration ID).

    Args:
        messages: List of can.Message objects for one motion packet.
    """
    motor_messages = [m for m in messages if m.arbitration_id != END_EFFECTOR_CAN_ID]

    if not motor_messages:
        return

    speeds = [(msg.data[1] << 8) + msg.data[2] for msg in motor_messages]
    reference_speed = sum(speeds) // len(speeds)

    if reference_speed == 0:
        return

    for msg in motor_messages:
        speed          = (msg.data[1] << 8) + msg.data[2]
        adjusted_speed = int((speed / reference_speed) * reference_speed)
        msg.data[1]    = (adjusted_speed >> 8) & 0xFF
        msg.data[2]    = adjusted_speed & 0xFF
        # Recompute CRC over data bytes 0-6 only (no arbitration ID)
        msg.data[7]    = sum(msg.data[:7]) & 0xFF


def can_send_messages(bus: can.interface.Bus, messages: List[can.Message]) -> None:
    """
    Sends all messages in a packet and waits for motor acknowledgements.

    Expected responses are arbitration IDs 1 and 2 (MKS motor ACK IDs).
    The end-effector (0x0D) does not send an ACK in this test setup.

    Args:
        bus:      Active CAN bus instance.
        messages: List of can.Message objects to send.
    """
    expected_responses = {1, 2}
    received_responses = set()

    for msg in messages:
        bus.send(msg)
        data_str = ", ".join([f"0x{b:02X}" for b in msg.data])
        print(f"Sent:     arbitration_id=0x{msg.arbitration_id:02X}  data=[{data_str}]")

    timeout    = 0.5
    start_time = time.time()

    while True:
        received_msg = bus.recv(timeout=3)

        if received_msg is not None:
            data_str = ", ".join([f"0x{b:02X}" for b in received_msg.data])
            print(f"Received: arbitration_id=0x{received_msg.arbitration_id:02X}  data=[{data_str}]")

            if received_msg.arbitration_id in expected_responses:
                received_responses.add(received_msg.arbitration_id)

        if received_responses == expected_responses:
            if all(
                received_msg.data[0] == 2 if received_msg is not None else False
                for received_msg in [bus.recv(timeout=0.1)] * len(expected_responses)
            ):
                print("All expected motors acknowledged with status 2. Moving to next packet.\n")
                break

        if time.time() - start_time > timeout:
            print("Timeout waiting for motor responses.\n")
            break


def main() -> None:
    """
    Reads a .txt file of CAN messages, groups into 7-line packets
    (6 motors + 1 end-effector), adjusts speeds, and sends over CAN.

    Usage: python send.py [filename.txt]
    Defaults to test.txt if no argument provided.
    """
    script_dir    = os.path.dirname(os.path.abspath(__file__))
    selected_file = sys.argv[1] if len(sys.argv) > 1 else "test.txt"
    file_path     = os.path.join(script_dir, selected_file)

    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    bus = can.interface.Bus(
        interface=CAN_INTERFACE,
        channel=CAN_CHANNEL,
        bitrate=CAN_BITRATE,
    )

    with open(file_path, "r") as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]

    message_sets = [lines[i:i+PACKET_SIZE] for i in range(0, len(lines), PACKET_SIZE)]

    for i, message_set in enumerate(message_sets):
        print(f"--- Packet {i+1}/{len(message_sets)} ---")
        messages = [parse_can_message(line) for line in message_set]
        adjust_speeds_within_packet(messages)
        can_send_messages(bus, messages)

    bus.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()