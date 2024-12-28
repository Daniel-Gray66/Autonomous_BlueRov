from pymavlink import mavutil
import time


def control_motors(connection, motor1=1500, motor2=1500, motor3=1500, motor4=1500, motor5=1500, motor6=1500):
    """
    Controls the motors of the BlueROV by sending RC channel override commands.

    Parameters:
    - connection: mavutil.mavlink_connection object, the connection to the BlueROV
    - motor1 to motor6: int, PWM values for the respective motors
      (1500 is neutral, >1500 for forward, <1500 for reverse/negative thrust)
    """
    try:
         print("Sending motor commands")
         connection.mav.rc_channels_override_send(
            connection.target_system,
            connection.target_component,
            motor1,  # Motor 1 (Roll)
            motor2,  # Motor 2 (Pitch)
            motor3,  # Motor 3 (Throttle)
            motor4,  # Motor 4 (Yaw)
            motor5,  # Motor 5
            motor6,  # Motor 6
            0, 0     # Unused channels
            )

    except Exception as e:
        print(f"An error occurred: {e}")


def stop_and_disarm(connection):
    """
    Stops all motors and disarms the BlueROV.

    Parameters:
    - connection: mavutil.mavlink_connection object, the connection to the BlueROV
    """
    try:
        print("Stopping all motors")
        connection.mav.rc_channels_override_send(
            connection.target_system,
            connection.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        time.sleep(5)
        
        print("Disarming the BlueROV")
        connection.arducopter_disarm()
        print("Disarmed")
        time.sleep(2)

    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage (replace 'master' with your connection object):
# master = mavutil.mavlink_connection('udp:192.168.2.1:14550')
# master.arducopter_arm()
# print("Armed")
# control_motors(master, motor1=1600, motor2=1500, motor3=1500, motor4=1500, motor5=1500, motor6=1500)
# stop_and_disarm(master)





# This function establishes a MAVLink connection and returns the connection object if successful, or False if not
def create_connectionlink():
    try:
        # The connection to the raspberry pi in the bluerov
        master = mavutil.mavlink_connection('udp:192.168.2.1:14550')

        #
        print("Waiting for heartbeat...")
        if master.wait_heartbeat(timeout=5):  
            print("Heartbeat received. Connection established.")

            # This is just printing out data requests.
            master.mav.request_data_stream_send(
                master.target_system, 
                master.target_component, 
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )

            return master  # Returning the connection link
        else:
            #if no heartbeat was detected then we return false
            print("Heartbeat not received. Connection failed.")
            return False
    except Exception as e:
        print(f"An error occurred: {e}")
        return False


# Example usage:
master = create_connectionlink()
master.arducopter_arm()

time.sleep(5)
#arm the robot

#Here is where we send out commands
print("starting the mortors")
control_motors(master, motor1=1600, motor2=1600, motor3=1600, motor4=1600, motor5=1600, motor6=1600)

time.sleep(5)

stop_and_disarm(master)






