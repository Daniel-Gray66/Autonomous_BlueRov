
"""
Example script to continuously move a BlueROV (ArduSub) in a loop
until user stops it with Ctrl+C.
"""

import sys
import time
from pymavlink import mavutil

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def create_connectionlink(connection_str='udp:192.168.2.1:14550', timeout=10):
    """
    Creates a MAVLink connection to the ROV.
    Returns a mavutil.mavlink_connection object if successful, or None if not.
    """
    try:
        print(f"Connecting to ROV on {connection_str}...")
        master = mavutil.mavlink_connection(connection_str)

        print("Waiting for heartbeat...")
        if master.wait_heartbeat(timeout=timeout):
            print("Heartbeat received. Connection established.")

            # Request data streams (optional, but helpful for telemetry)
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,  # rate (Hz)
                1,  # start or stop the data stream
            )
            return master
        else:
            print("Heartbeat not received. Connection failed.")
            return None

    except Exception as e:
        print(f"An error occurred while connecting: {e}")
        return None


def arm_vehicle(master, timeout=10):
    """
    Attempts to arm the vehicle within `timeout` seconds.
    Returns True if armed, False otherwise.
    """
    print("Sending arm command...")
    master.arducopter_arm()

    start_time = time.time()
    while time.time() - start_time < timeout:
        # Check if motors are armed
        if master.motors_armed():
            print("Vehicle is armed.")
            return True

        # Optionally, check the COMMAND_ACK message to see if it was accepted
        msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Arming accepted by autopilot.")
                return True
            else:
                print(f"Arming rejected (result code {msg.result}).")
                return False

        time.sleep(0.5)

    print("Arming timed out. Vehicle did not arm.")
    return False


def disarm_vehicle(master):
    """
    Disarms the vehicle.
    """
    print("Sending disarm command...")
    master.arducopter_disarm()
    time.sleep(1)  # Let the disarm command process
    if not master.motors_armed():
        print("Vehicle is now disarmed.")
    else:
        print("Warning: Vehicle still armed.")


def control_motors(master, roll=1500, pitch=1500, throttle=1500, yaw=1500, ch5=1500, ch6=1500, ch7 =1900):
    """
    Send RC override commands to control the motors.

    By default, 1500 is neutral. 1000-2000 is the typical PWM range.
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        roll,     # Channel 1: Roll
        pitch,    # Channel 2: Pitch
        throttle, # Channel 3: Throttle
        yaw,      # Channel 4: Yaw
        ch5,      # Channel 5
        ch6,      # Channel 6
        ch7,        # Channel 7 (unused)
        0         # Channel 8 (unused)
    )
    

    time.sleep(4)
    



# -----------------------------------------------------------------------------
# Main script logic
# -----------------------------------------------------------------------------
"""
def main():
    # 1. Create connection
    master = create_connectionlink()
    if not master:
        sys.exit("Failed to establish MAVLink connection. Exiting.")

    # 2. Arm vehicle
    if not arm_vehicle(master):
        sys.exit("Vehicle refused to arm. Exiting.")

    print("Sending continuous motor commands...")
    print("Press Ctrl+C to stop and disarm.")

    

    try:
        while True:
            # Example: Provide forward thrust on the pitch channel (1500 is neutral; 1600 is slight forward).
            # Adjust as needed for your thruster configuration and desired movement:
            control_motors(
                master,
                roll=1500,
                pitch=1500,   # top right
                throttle=1500,
                yaw=1500,#all of the bottoms ones
                ch5=1500,
                ch6=1500,
                ch7 = 1500,
            )

            

            # Sleep a bit before sending the next command
            time.sleep(0.2)

            # NOTE: Because we are in a continuous loop, the thrusters will stay at this setting
            # until we break out (Ctrl+C) or the script otherwise ends.
    except KeyboardInterrupt:
        print("\nReceived Ctrl+C, stopping motors and disarming...")
        
    finally:
        # Send neutral values to stop all motors
        control_motors(master, 1500, 1500, 1500, 1500, 1500, 1500,1500)

        # Disarm the vehicle
        disarm_vehicle(master)

        print("Exiting script.")


if __name__ == "__main__":
    main()
"""