#!/usr/bin/env python

import argparse
import time

from motorbridge_smart_servo import FashionStarServo, ServoBusError, ServoMonitor


DEFAULT_JOINT_IDS = {
    "shoulder_pan": 0,
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_yaw": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read raw reBot Arm 102 servo angles without LeRobot calibration."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument(
        "--interval", type=float, default=0.2, help="Polling interval in seconds"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    with FashionStarServo(args.port, baudrate=args.baudrate) as bus:
        # Initialize servos: ping, unlock, and reset each multi-turn counter.
        # The current motorbridge-smart-servo SDK does not use the old
        # fashionstar_uart_sdk broadcast ResetLoop API.
        for joint_name, servo_id in DEFAULT_JOINT_IDS.items():
            if not bus.ping(servo_id):
                print(f"WARNING: {joint_name} servo id={servo_id} did not respond to ping")
            bus.unlock(servo_id)
            time.sleep(0.01)
            bus.reset_multi_turn(servo_id)

        id_to_name = {servo_id: joint_name for joint_name, servo_id in DEFAULT_JOINT_IDS.items()}
        servo_ids = list(DEFAULT_JOINT_IDS.values())

        try:
            print("Reading raw servo angles. Press Ctrl+C to stop.")
            while True:
                monitor_data: dict[int, ServoMonitor | None] = bus.sync_monitor(servo_ids)
                values = []
                for servo_id in servo_ids:
                    joint_name = id_to_name[servo_id]
                    state = monitor_data.get(servo_id)
                    if state is None:
                        values.append(f"{joint_name}=<missing>")
                    else:
                        values.append(f"{joint_name}={state.angle_deg:8.2f}")
                print("  ".join(values), flush=True)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            pass
        except ServoBusError as exc:
            print(f"ERROR: servo bus read failed: {exc}")
            print("[EMERGENCY STOP] Hold the follower arm and cut power if needed.")
            raise


if __name__ == "__main__":
    main()
