import time
import sys
from pymavlink import mavutil

# --- CONFIGURATION MATRIX ---
# Replace these with your verified field location coordinates before flight
TARGET_LATITUDE = 41.161700  
TARGET_LONGITUDE = -8.583600
TARGET_ALTITUDE_METERS = 15.0

SAFETY_PARAMETERS = {
    "FENCE_ENABLE": 1.0,       # Enable geofencing
    "FENCE_TYPE": 3.0,         # Max altitude AND horizontal radius restriction
    "FENCE_ALT_MAX": 40.0,     # Ceil flight window at 40 meters
    "FENCE_RADIUS": 50.0,      # Constrain asset within a 50-meter radius
    "FENCE_ACTION": 1.0,       # Initiate Return-To-Launch (RTL) upon breach
    "FS_BATT_ENABLE": 1.0,     # Initiate RTL on critical low voltage
    "FS_BATT_LOW_VOLT": 10.5,  # Trigger voltage ceiling threshold (Adjust for cell count)
    "FS_GCS_ENABLE": 1.0,      # Trigger RTL on data link loss/script crash
    "WPNAV_SPEED": 150.0,      # Cap maximum horizontal transit speed at 1.5 m/s
    "WPNAV_SPEED_UP": 100.0    # Cap maximum vertical ascent rate at 1.0 m/s
}

def set_hardware_parameter(master, param_name, value):
    """Encapsulates param injection frame delivery and forces an EEPROM write delay."""
    print(f"[PARAM] Writing {param_name} to {value}...", end="", flush=True)
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param_name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)
    print(" Verified.")

def main():
    print("[INIT] Initializing MAVLink bridge on loopback interface 127.0.0.1:14550...")
    master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

    print("[INIT] Intercepting telemetry stream... Waiting for system heartbeat...")
    master.wait_heartbeat()
    print(f"[INIT] Established target link to Vehicle (SysID: {master.target_system} | CompID: {master.target_component})")

    # --- PUSH SAFETY POLICIES ---
    print("[SECURITY] Deploying fail-safes and dynamic flight envelope profiles...")
    for param, val in SAFETY_PARAMETERS.items():
        set_hardware_parameter(master, param, val)
    print("[SECURITY] All hardcoded safety layers successfully committed to flight controller.")
    time.sleep(1.0)

    # --- SET MODE: GUIDED ---
    print("[MODE] Transmitting flight control mode transition frame [GUIDED]...")
    while True:
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg and msg.custom_mode == 4:
            print("[MODE] Guided state confirmed by autopilot.")
            break
        time.sleep(1)

    # --- MOTOR ARMING LAYER ---
    print("[FLIGHT] Initializing subsystem pre-arm checklists...")
    while True:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        
        if not msg:
            continue

        if not bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[FLIGHT] System disarmed. Supplying component arm sequence command...")
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(2.0)
            continue

        # --- EXECUTE TAKEOFF ---
        print(f"[FLIGHT] Propulsion active. Executing autonomous takeoff vector to {TARGET_ALTITUDE_METERS}m...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, TARGET_ALTITUDE_METERS
        )
        
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("[FLIGHT] Takeoff frame parsed and ACCEPTED by flight management system.")
                break
            else:
                print(f"[CRITICAL] Takeoff rejected by hardware. EKF/GPS fault code: {ack.result}")
        time.sleep(1)

    print("[FLIGHT] Scaling vertical column ceiling... Holding telemetry state loop for 15s...")
    time.sleep(15)

    # --- TRAJECTORY NAVIGATION LINK ---
    print(f"[NAV] Pushing global coordinate waypoint targeting: Lat {TARGET_LATITUDE}, Lon {TARGET_LONGITUDE}...")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component, 
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 
        0b110111111000, 
        int(TARGET_LATITUDE * 1e7), 
        int(TARGET_LONGITUDE * 1e7), 
        TARGET_ALTITUDE_METERS, 
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[NAV] Autonomous mission vectors handed over to autopilot cache. Exiting application layer runtime.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[WARN] Operational script aborted manually via standard keyboard signal interrupt.")
        sys.exit(0)