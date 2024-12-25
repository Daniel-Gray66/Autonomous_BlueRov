from pymavlink import mavutil
import time



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

#rov is the connection between the bluerov and my laptop and the other parameters are the rotors.

def fishy_controls(connection, rot1, rot2, rot3, rot4, rot5, rot6):
    """
    Controls the BlueROV's movement and other features based on input parameters.
    :param connection: MAVLink connection object
    :param rot1 - rot6: PWM values for the six motors or actuators
    """
    try:
        # Turn on lights (assumes channel 10 controls lights)
        print("Turning on the lights...")
        connection.set_servo(10, 1500)  # PWM value to turn on lights
        time.sleep(5)
        
        print("Turning off the lights...")
        connection.set_servo(10, 1100)  # PWM value to turn off lights

        # Send RC override commands for movement
        print("Sending movement commands...")
        connection.mav.rc_channels_override_send(
            connection.target_system,
            connection.target_component,
            rot1,  # Channel 1 (e.g., throttle)
            rot2,  # Channel 2 (e.g., yaw)
            rot3,  # Channel 3 (e.g., pitch)
            rot4,  # Channel 4 (e.g., roll)
            rot5,  # Channel 5 (optional)
            rot6,  # Channel 6 (optional)
            0, 0   # Remaining unused channels
        )

        time.sleep(3)  # Keep the movement active for 3 seconds

        # Stop movement by resetting channels to neutral
        print("Stopping movement...")
        connection.mav.rc_channels_override_send(
            connection.target_system,
            connection.target_component,
            1500,  # Neutral for Channel 1
            1500,  # Neutral for Channel 2
            1500,  # Neutral for Channel 3
            1500,  # Neutral for Channel 4
            0, 0, 0, 0  # Remaining unused channels
        )

        print("All operations completed.")
    
    except Exception as e:
        print(f"An error occurred: {e}")




#master = mavutil.mavlink_connection('udp:192.168.2.1:14550')
master = create_connectionlink()

#master.set_mode("MANUAL")
master.arducopter_arm()
print("armed")

time.sleep(3)   

# Parameters that control the motors of the bluerov
neutral = 1500
forward_pwn = 1600

print("Turning on")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0,
    forward_pwn,
    0,  # vertical
    0,  #(yaw)
    0,0,0,0
    )

#applying forward thrust for 5
time.sleep(3)

print("Turning off")

master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0,0,0,0,0,0,0,0,)

time.sleep(1)


#####
master.arducopter_disarm()


print("disarm")
master.close()
