#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from pymavlink import mavutil

@dataclass
class State:
    x: float = float('nan')
    y: float = float('nan')
    z: float = float('nan')
    vx: float = float('nan')
    vy: float = float('nan')
    vz: float = float('nan')
    yaw: float = float('nan')
    roll: float = float('nan')
    pitch: float = float('nan')
    last_lpos: float = 0.0
    last_att: float = 0.0


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def euler_from_quat(q1, q2, q3, q4):
    # MAVLink ATTITUDE_QUATERNION uses w,x,y,z in q1..q4
    w, x, y, z = q1, q2, q3, q4
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def recv_state(mav, state: State, timeout=0.05):
    end = time.time() + timeout
    while time.time() < end:
        msg = mav.recv_match(blocking=True, timeout=max(0, end - time.time()))
        if not msg:
            break
        t = msg.get_type()
        if t == 'LOCAL_POSITION_NED':
            state.x = float(msg.x)
            state.y = float(msg.y)
            state.z = float(msg.z)
            state.vx = float(msg.vx)
            state.vy = float(msg.vy)
            state.vz = float(msg.vz)
            state.last_lpos = time.time()
        elif t == 'ATTITUDE_QUATERNION':
            r, p, y = euler_from_quat(msg.q1, msg.q2, msg.q3, msg.q4)
            state.roll, state.pitch, state.yaw = r, p, y
            state.last_att = time.time()
        elif t == 'ATTITUDE':
            state.roll = float(msg.roll)
            state.pitch = float(msg.pitch)
            state.yaw = float(msg.yaw)
            state.last_att = time.time()
    return state


def send_offboard_heartbeat(mav):
    mav.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        int(0b100111111000),
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0,
    )


def send_pos_yaw(mav, x, y, z, yaw, yaw_rate=0.0):
    type_mask = int(
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    )
    mav.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw, yaw_rate,
    )


def set_offboard_mode(mav):
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        6.0,  # PX4 custom main mode: OFFBOARD
        0.0,
        0.0, 0.0, 0.0, 0.0,
    )


def main():
    mav = mavutil.mavlink_connection('udpin:127.0.0.1:14550', autoreconnect=False)
    hb = mav.wait_heartbeat(timeout=5)
    if not hb:
        raise SystemExit('no heartbeat')
    state = State()
    t0 = time.time()
    while time.time() - t0 < 3:
        recv_state(mav, state, timeout=0.1)
        send_offboard_heartbeat(mav)
        time.sleep(0.05)

    # arm + offboard
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(0.5)
    for _ in range(20):
        send_offboard_heartbeat(mav)
        time.sleep(0.05)
    set_offboard_mode(mav)
    print('OFFBOARD requested')

    # climb to hover
    target_x = 0.0 if math.isnan(state.x) else state.x
    target_y = 0.0 if math.isnan(state.y) else state.y
    hover_z = -3.5
    for _ in range(240):
        recv_state(mav, state, timeout=0.02)
        send_pos_yaw(mav, target_x, target_y, hover_z, 0.0, 0.0)
        time.sleep(0.05)

    # yaw probe
    start = time.time()
    samples = []
    while time.time() - start < 12.0:
        recv_state(mav, state, timeout=0.02)
        t = time.time() - start
        yaw_ref = 0.6 * math.sin(2.0 * math.pi * 0.08 * t)
        yaw_rate_ref = 0.6 * 2.0 * math.pi * 0.08 * math.cos(2.0 * math.pi * 0.08 * t)
        send_pos_yaw(mav, target_x, target_y, hover_z, yaw_ref, yaw_rate_ref)
        if not math.isnan(state.yaw):
            samples.append((t, yaw_ref, state.yaw, state.z, state.roll, state.pitch))
        time.sleep(0.05)

    max_err = max(abs(wrap_pi(y - r)) for _, r, y, *_ in samples) if samples else float('nan')
    yaw_span = max(y for _, _, y, *_ in samples) - min(y for _, _, y, *_ in samples) if samples else float('nan')
    ref_span = max(r for _, r, *_ in samples) - min(r for _, r, *_ in samples) if samples else float('nan')
    print(f'yaw_probe_summary ref_span={ref_span:.3f} yaw_span={yaw_span:.3f} max_err={max_err:.3f}')

    # safe disarm
    for _ in range(20):
        send_pos_yaw(mav, target_x, target_y, hover_z, 0.0, 0.0)
        time.sleep(0.05)
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              0, 0, 21196, 0, 0, 0, 0, 0)

if __name__ == '__main__':
    main()
