#!/usr/bin/env python

#sgillen - this program serves as a node that offers the arduino up to the rest of the ros system.

import serial, time, sys, select
import rospy
from std_msgs.msg import Int64, Float64, String, Float64MultiArray, Byte
from std_srvs.srv import Empty, EmptyResponse
from aqbar.msg import falconForces, falconPos


 
X_MAX = 11; X_MIN = 5.5
Y_MAX = 6 ; Y_MIN = -2.5
Z_MAX = 90; Z_MIN = 0

#device = '/dev/ttyACM1' # TODO
device = '/dev/ttyUSB0'
pose_cmds = [0,0,0]


# When this gets flipped, send shutdown signal
shutdown_flag = False

#reads a command from stdin
def read_cmd_stdin():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      line = sys.stdin.readline()
      line = line.rstrip()
      if line:
        num_bytes = ser.write(line)
        print  "bytes sent =", num_bytes


#sends an array of ints to the thrusters using the agreed upon protocol
#the actual over the wire value is m,x,y,z!
def send_pose_cmds(pose_cmd):
    cmd_str = "m"
    for cmd in pose_cmds:
        cmd_str += (",")
        cmd_str += (str(cmd))

    cmd_str += ("!")
    ser.write(cmd_str)
    #print "arduino return", ser.readline()
    ##TODO parse return value


##------------------------------------------------------------------------------
# callbacks
def shutdown_thrusters(srv):
    global shutdown_flag
    shutdown_flag = True
    return EmptyResponse()

def pose_callback(msg):
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.X)
    #    print "I heard %s" %msg.X
    global pose_cmds

    # map falcon workspace to lynx workspace
    pose_cmds[0] = str((msg.X/.2 * (X_MAX - X_MIN)) + X_MIN)
    pose_cmds[1] = str((msg.Y/.2 * (Y_MAX - Y_MIN)) + Y_MIN)
    pose_cmds[2] = str((msg.Z/.2 * (Z_MAX - Z_MIN)) + Z_MIN)


    pose_cmds[0] = pose_cmds[0][:4]
    pose_cmds[1] = pose_cmds[0][:4]
    pose_cmds[1] = pose_cmds[0][:4]

    # truncate the strings that we send

    print "pose_cmds: " , pose_cmds[0], pose_cmds[1], pose_cmds[2]

##------------------------------------------------------------------------------
if __name__ == '__main__':

    #!!! this also restarts the arduino!

    print "trying to connect to arduino"
   # Keep trying to open serial
    while True:
        try:
            ser = serial.Serial(device,9600, timeout=0,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            break
        except:
            time.sleep(0.25)
            continue


    print "found it!"
    time.sleep(3)

    rospy.init_node('lynx_node', anonymous=False)


    # There are smarter ways to do this...
    # Could do it bitwise, could have an array of bools
    # None of that really matters though for this use case, and this
    # will be easier to read and modify I think
    sensor_pub = rospy.Publisher('contact_sensor', Byte, queue_size = 10)
        
    pose_sub = rospy.Subscriber('falconPos', falconPos, pose_callback)
    
    rate = rospy.Rate(5) #Hz

    while not rospy.is_shutdown():

        send_pose_cmds(pose_cmds)
        #print "writing pose"

        #ser.write('c!')
        temp = ser.readline()
        print(temp)

        # get thruster cmd
        #x = ser.readline().strip()
        #print x


        # if x != '':
        #     msg[x[0]] = x[1:]

        # x = ser.readline().strip()
        # if x != '':
        #     msg[x[0]] = x[1:]
        #status = ser.readline().strip()

        #print pose_cmds
        
        rate.sleep()
