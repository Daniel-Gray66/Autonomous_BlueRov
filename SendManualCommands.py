# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udp:192.168.2.1:14550')
# Wait a heartbeat before sending commands
print("waiting for heartbeat")
master.wait_heartbeat()


master.mav.request_data_stream_send( 
    master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
for i in range(10):
    msg = master.recv_match(blocking = True)
    print(msg)



#we're going to arm the device
master.arducopter_arm()
print("The robot is turning on ")


# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].



master.mav.manual_control_send(
    master.target_system,
    500,
    -500,
    250,
    500,
    0)









print("turning off")

master.arducopter_disarm()
