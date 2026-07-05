import time
from pymavlink import mavutil

print("[SCRIPT] Connecting to Copter on 127.0.0.1:14550...", flush=True)
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

print("[SCRIPT] Waiting for telemetry heartbeat...", flush=True)
master.wait_heartbeat()
print("[SCRIPT] Connected! Initializing master flight controller...", flush=True)

# 1. LOOP UNTIL MODE IS SUCCESSFULLY GUIDED
print("[SCRIPT] Switching flight mode to GUIDED...", flush=True)
while True:
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
    if msg and msg.custom_mode == 4:
        print("[SCRIPT] Mode successfully verified as GUIDED!", flush=True)
        break
    print("[SCRIPT] Mode switch pending... retrying...", flush=True)
    time.sleep(1)

# 2. MASTER ARM & TAKEOFF LOOP
print("[SCRIPT] Entering Arm & Takeoff orchestration loop...", flush=True)
while True:
    # Send GCS heartbeat to keep communication link active
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    
    # CRITICAL: Wait specifically for a FRESH heartbeat to evaluate true arm state
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.5)
    
    if not msg:
        print("[SCRIPT] Heartbeat timeout... checking link...", flush=True)
        continue

    is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    
    if not is_armed:
        print("[SCRIPT] Vehicle is DISARMED. Sending arm command...", flush=True)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        # Give it a tiny window to change state before checking the next heartbeat
        time.sleep(1)
        continue

    # If we pass the check, the vehicle is verified armed.
    print("[SCRIPT] Vehicle verified ARMED! Attempting immediate takeoff to 30m...", flush=True)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, 30
    )
    
    # Check if the takeoff command was accepted or denied
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[SCRIPT] Takeoff command ACCEPTED by autopilot!", flush=True)
            break
        else:
            print(f"[SCRIPT] Takeoff denied by autopilot (Result Code: {ack.result}). Resetting state...", flush=True)
    
    time.sleep(1)

print("[SCRIPT] Climbing safely... holding altitude check for 15 seconds...", flush=True)
time.sleep(15)

# 3. Direct route command towards INESC TEC
print("[SCRIPT] Flying to INESC TEC...", flush=True)
master.mav.set_position_target_global_int_send(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b110111111000, int(41.1780 * 1e7), int(-8.5980 * 1e7), 30, 0, 0, 0, 0, 0, 0, 0, 0)
print("[SCRIPT] Success! Mission running autonomously.", flush=True)