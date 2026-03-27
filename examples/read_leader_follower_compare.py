#!/usr/bin/env python

import argparse
import os
import time
from pathlib import Path

from lerobot_teleoperator_fasionstar_pipermate import (
    FasionStarPiperMateLeader,
    FasionStarPiperMateLeaderConfig,
)
from lerobot_robot_seeed_b601 import (
    SeeedB601DMFollower,
    SeeedB601DMFollowerConfig,
    SeeedB601RSFollower,
    SeeedB601RSFollowerConfig,
)


class PassiveSeeedB601DMFollower(SeeedB601DMFollower):
    """Read-only DM follower variant for manual comparison/debugging."""

    def connect(self, calibrate: bool = False) -> None:
        super().connect(calibrate=calibrate)
        self.disable_torque()


class PassiveSeeedB601RSFollower(SeeedB601RSFollower):
    """Read-only RS follower variant for manual comparison/debugging."""

    def connect(self, calibrate: bool = False) -> None:
        super().connect(calibrate=calibrate)
        self.disable_torque()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read PiperMate leader and B601 follower positions side by side."
    )
    parser.add_argument("--leader-port", required=True, help="PiperMate serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--leader-id", default="pipermate_leader")
    parser.add_argument("--leader-baudrate", type=int, default=1_000_000)
    parser.add_argument(
        "--leader-calibration-dir",
        type=Path,
        default=None,
        help="Optional LeRobot calibration directory override for the PiperMate leader",
    )
    parser.add_argument("--follower-port", required=True, help="B601 CAN port, e.g. can0 or /dev/ttyACM0")
    parser.add_argument("--follower-id", default="b601_follower")
    parser.add_argument("--follower-type", choices=["dm", "rs"], default="dm")
    parser.add_argument("--follower-can-adapter", default="socketcan")
    parser.add_argument("--follower-dm-serial-baud", type=int, default=921600)
    parser.add_argument("--interval", type=float, default=0.2, help="Polling interval in seconds")
    return parser.parse_args()


def make_follower(args: argparse.Namespace):
    if args.follower_type == "dm":
        config = SeeedB601DMFollowerConfig(
            id=args.follower_id,
            port=args.follower_port,
            can_adapter=args.follower_can_adapter,
            dm_serial_baud=args.follower_dm_serial_baud,
            cameras={},
        )
        return PassiveSeeedB601DMFollower(config)

    config = SeeedB601RSFollowerConfig(
        id=args.follower_id,
        port=args.follower_port,
        can_adapter=args.follower_can_adapter,
        dm_serial_baud=args.follower_dm_serial_baud,
        cameras={},
    )
    return PassiveSeeedB601RSFollower(config)


def main() -> None:
    args = parse_args()

    leader = FasionStarPiperMateLeader(
        FasionStarPiperMateLeaderConfig(
            id=args.leader_id,
            port=args.leader_port,
            baudrate=args.leader_baudrate,
            calibration_dir=args.leader_calibration_dir,
        )
    )
    follower = make_follower(args)

    leader.connect(calibrate=False)
    follower.connect(calibrate=False)

    try:
        if not leader.is_calibrated:
            raise RuntimeError(
                "No PiperMate calibration file found. Run examples/calibrate.py first."
            )

        print("Reading leader/follower positions side by side. Press Ctrl+C to stop.")
        while True:
            leader_action = leader.get_action()
            follower_obs = follower.get_observation()
            os.system("clear")
            print("Reading leader/follower positions side by side. Press Ctrl+C to stop.\n")
            print(f"{'joint':<16} {'leader':>10} {'follower':>10} {'delta':>10}")
            print(f"{'-' * 16} {'-' * 10} {'-' * 10} {'-' * 10}")
            for joint in leader.motor_names:
                leader_pos = leader_action[f"{joint}.pos"]
                follower_pos = follower_obs[f"{joint}.pos"]
                delta = follower_pos - leader_pos
                print(f"{joint:<16} {leader_pos:10.2f} {follower_pos:10.2f} {delta:10.2f}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        follower.disconnect()
        leader.disconnect()


if __name__ == "__main__":
    main()
