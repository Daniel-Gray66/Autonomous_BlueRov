from pymavlink import mavutil
import time

#This function is meant to create the mavlink connection and will return true if the heart beat and recieved and false if it isn't
#def create_connectionlink()
    #master = mavutil.mavlink_connection('udp:192.168.2.1:14550')
    #if(master.wait_heartbeat())
        #master.mav.request_data_stream_send( 
        #    master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
       # for i in range(10):
      #      msg = master.recv_match(blocking = True)
     #       print(msg)
    #    return master
   # else 
  #  return False

#This function will be responsible for moving all the components inside the blue rov
#def control_rov(int,int,int,int,int,int,int)
 #   master = mavutil.mavlink_connection('udp:192.168.2.1:14550')







master = mavutil.mavlink_connection('udp:192.168.2.1:14550')

master.wait_heartbeat()
print("Heartbeat Received. System and component: ", master.target_system,master.target_component)

master.mav.request_data_stream_send( 
    master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
for i in range(10):
    msg = master.recv_match(blocking = True)
    print(msg)

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



master.arducopter_disarm()

print("disarm")
master.close()
