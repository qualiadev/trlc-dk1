#   Copyright 2025 The Robot Learning Company UG (haftungsbeschränkt). All rights reserved.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.

"""Measure a DK1 gripper's closed position (``gripper_closed_pos``).

Why: on connect the gripper auto-opens until the hard stop and the motor is
zeroed there, so OPEN is always 0.0 — but the CLOSED position in raw motor
radians varies between gripper assemblies. A wrong value means commanded
``gripper.pos=1.0`` either doesn't fully close (closed_pos too negative) or
stalls against the hard stop at full torque (not negative enough), and the
normalized observation is wrong by the same factor.

How it works:
  1. Connects to the arm. NOTE: the standard connect routine drives the
     gripper OPEN and zeroes it there — keep fingers/objects clear.
  2. Disables torque on ALL motors (arm becomes limp — support it or have it
     resting in a safe pose).
  3. You squeeze the gripper fully closed BY HAND and hold it there.
  4. The script streams the raw gripper position and tracks the minimum
     (most negative) value seen. Ctrl+C when the reading is stable.
  5. It prints the measured value and the exact config line to set.

Usage:
    python examples/calibrate_gripper.py --port /dev/ttyACM1
    python examples/calibrate_gripper.py --port /dev/ttyACM3   # other arm
"""

import argparse
import time

from lerobot_robot_trlc_dk1.follower import DK1Follower, DK1FollowerConfig


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True, help="Follower serial port, e.g. /dev/ttyACM1")
    args = p.parse_args()

    print(f"Connecting to follower on {args.port} ...")
    print("NOTE: the gripper will auto-OPEN and zero itself — keep it clear.")
    # pos_vel mode: we need direct bus access to read raw motor positions
    # with torque off; impedance mode hides the bus behind trlc_dk1_control.
    follower = DK1Follower(DK1FollowerConfig(port=args.port, control_mode="pos_vel"))
    follower.connect()

    # Make the whole arm limp so the gripper can be moved by hand.
    for motor in follower._motors.values():
        follower._control.disable(motor)
    print("\nTorque disabled on all motors — the arm is limp, support it if needed.")
    print("Now squeeze the gripper FULLY CLOSED by hand and hold.")
    print("Position streams below; Ctrl+C when the value is stable.\n")

    min_pos = 0.0
    try:
        while True:
            follower._control.refresh_motor_status(follower._motors["gripper"])
            pos = follower._motors["gripper"].getPosition()
            min_pos = min(min_pos, pos)
            print(f"\rgripper raw pos: {pos:+8.4f} rad   (min seen: {min_pos:+8.4f})",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        follower.disconnect()

    print(f"\n\nMeasured gripper_closed_pos: {min_pos:.4f}")
    print("\nSet it in your robot config:")
    print(f"  single arm : DK1FollowerConfig(..., gripper_closed_pos={min_pos:.4f})")
    print(f"  bimanual   : BiDK1FollowerConfig(..., left_gripper_closed_pos=... ,")
    print(f"                                      right_gripper_closed_pos=...)")
    print(f"  CLI        : --robot.gripper_closed_pos={min_pos:.4f}")
    print("\nSanity: value should be NEGATIVE, typically around -4 to -5.5 rad.")
    if min_pos > -1.0:
        print("WARNING: measured value is suspiciously close to 0 — was the "
              "gripper actually closed? Re-run and squeeze it fully shut.")


if __name__ == "__main__":
    main()
