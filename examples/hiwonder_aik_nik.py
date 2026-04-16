import time
import traceback

import numpy as np

from funrobo_hiwonder.core.hiwonder import HiwonderRobot
from funrobo_kinematics.projects.mp2.five_dof import FiveDOFRobot
import funrobo_kinematics.core.utils as ut


DEBUG_RRMC = True
DEBUG_LOG_INTERVAL = 0.5

VELOCITY_SCALE = 3.0
VEL_DEADZONE = 0.001
JOINT_DELTA_WARN_DEG = 5.0
JOINT_DELTA_CLAMP_DEG = 15.0

NORMALIZE_VELOCITY = False
FIXED_CARTESIAN_SPEED = 0.15

USE_ANALYTICAL_IK = True

def solve_ik(model, ee, init_joint_values):
    if USE_ANALYTICAL_IK:
        result = model.calc_inverse_kinematics(ee, init_joint_values)
    else:
        result = model._newton_raphson_step(init_joint_values, np.array([ee.x, ee.y, ee.z]), tol=0.001, max_iter=30)

    if result is None or len(result) == 0:
        return None
    return result


def main():
    robot = None
    try:
        robot = HiwonderRobot()
        model = FiveDOFRobot()

        control_hz = 20
        dt = 1 / control_hz
        last_debug_log = 0.0
        tick = 0

        print(f"[INFO] Control loop running at {control_hz} Hz  (dt={dt:.4f}s)")
        print(f"[INFO] VELOCITY_SCALE={VELOCITY_SCALE}  JOINT_DELTA_WARN_DEG={JOINT_DELTA_WARN_DEG}")
        print(f"[INFO] NORMALIZE_VELOCITY={NORMALIZE_VELOCITY}  FIXED_CARTESIAN_SPEED={FIXED_CARTESIAN_SPEED}")
        print(f"[INFO] IK MODE: {'ANALYTICAL' if USE_ANALYTICAL_IK else 'NUMERICAL'}")
        print(f"[INFO] FiveDOFRobot joint_limits: {model.joint_limits}")
        print(f"[INFO] FiveDOFRobot joint_vel_limits: {getattr(model, 'joint_vel_limits', 'NOT SET')}")

        tracked_joint_values = robot.get_joint_values()

        print(f"[DIAG] curr_arm_rad (home): {[f'{np.deg2rad(j):.4f}' for j in tracked_joint_values[:5]]}")
        print(f"[DIAG] curr_joint_values (home, deg): {tracked_joint_values}")

        while True:
            t_start = time.time()
            tick += 1

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                cmd = robot.gamepad.cmdlist[-1]

                if cmd.arm_home:
                    print("[INFO] arm_home command received — moving to home position")
                    robot.move_to_home_position()
                    tracked_joint_values = robot.get_joint_values()

                curr_arm_rad = [np.deg2rad(j) for j in tracked_joint_values[:5]]

                raw_vel = [
                    VELOCITY_SCALE * cmd.arm_vx,
                    VELOCITY_SCALE * cmd.arm_vy,
                    VELOCITY_SCALE * cmd.arm_vz,
                ]
                vel_mag = sum(v * v for v in raw_vel) ** 0.5

                if NORMALIZE_VELOCITY and vel_mag > VEL_DEADZONE:
                    scale = FIXED_CARTESIAN_SPEED / vel_mag
                    vel = [v * scale for v in raw_vel]
                    vel_mag = FIXED_CARTESIAN_SPEED
                else:
                    vel = raw_vel

                if vel_mag > VEL_DEADZONE:
                    # Use RRMC to get a warm-start seed near the current config
                    rrmc_seed = model.calc_velocity_kinematics(curr_arm_rad, vel, dt=dt)

                    curr_ee, _ = model.calc_forward_kinematics(curr_arm_rad)
                    target_ee = ut.EndEffector()
                    target_ee.x = curr_ee.x + vel[0] * dt
                    target_ee.y = curr_ee.y + vel[1] * dt
                    target_ee.z = curr_ee.z + vel[2] * dt

                    ik_result = solve_ik(model, target_ee, rrmc_seed)

                    if ik_result is not None:
                        joint_delta_rad = [ik_result[i] - curr_arm_rad[i] for i in range(5)]
                        max_delta_deg = max(abs(np.rad2deg(d)) for d in joint_delta_rad)
                        if max_delta_deg > JOINT_DELTA_CLAMP_DEG:
                            print(f"[WARN] tick={tick} IK solution rejected: max delta={max_delta_deg:.1f} deg, falling back to RRMC seed")
                            new_arm_rad = rrmc_seed
                        else:
                            new_arm_rad = ik_result
                    else:
                        print(f"[WARN] tick={tick} IK failed, falling back to RRMC seed")
                        new_arm_rad = rrmc_seed

                    new_arm_deg = [np.rad2deg(j) for j in new_arm_rad]
                    new_joint_values = new_arm_deg + [tracked_joint_values[5]]

                    joint_delta_rad = [new_arm_rad[i] - curr_arm_rad[i] for i in range(5)]
                    joint_delta_deg = [np.rad2deg(d) for d in joint_delta_rad]
                    delta_mag = sum(d * d for d in joint_delta_deg) ** 0.5

                    for i, delta in enumerate(joint_delta_deg):
                        if abs(delta) > JOINT_DELTA_WARN_DEG:
                            print(
                                f"[WARN] tick={tick}  joint {i+1} delta={delta:+.2f} deg in one tick! "
                                f"curr={np.rad2deg(curr_arm_rad[i]):.2f}  new={new_arm_deg[i]:.2f}"
                            )

                    now = time.time()
                    if DEBUG_RRMC and (now - last_debug_log) >= DEBUG_LOG_INTERVAL:
                        try:
                            J = getattr(model, '_last_jacobian', None)
                            if J is None:
                                J = model.jacobian(curr_arm_rad)
                            cond = np.linalg.cond(J)
                            cond_str = f"{cond:.1f}"
                            if cond > 100:
                                cond_str += "  *** NEAR SINGULAR ***"
                        except Exception as je:
                            cond_str = f"ERROR ({je})"

                        print(
                            f"[DEBUG] tick={tick}  IK={'AIK' if USE_ANALYTICAL_IK else 'NIK'}\n"
                            f"  vel       = [{vel[0]:+.3f}, {vel[1]:+.3f}, {vel[2]:+.3f}] m/s  |vel|={vel_mag:.3f}\n"
                            f"  curr_deg  = {[f'{np.rad2deg(r):+.2f}' for r in curr_arm_rad]}\n"
                            f"  new_deg   = {[f'{d:+.2f}' for d in new_arm_deg]}\n"
                            f"  delta_deg = {[f'{d:+.2f}' for d in joint_delta_deg]}  |delta|={delta_mag:.2f} deg\n"
                            f"  J cond#   = {cond_str}"
                        )
                        last_debug_log = now

                    robot.set_joint_values(new_joint_values, duration=dt, radians=False)
                    tracked_joint_values = new_joint_values

            elapsed = time.time() - t_start
            remaining_time = dt - elapsed
            if remaining_time > 0:
                time.sleep(remaining_time)
            elif elapsed > dt * 1.5:
                print(f"[WARN] tick={tick} loop overran: elapsed={elapsed*1000:.1f}ms > dt={dt*1000:.1f}ms")

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        if robot is not None:
            robot.shutdown_robot()


if __name__ == "__main__":
    main()
