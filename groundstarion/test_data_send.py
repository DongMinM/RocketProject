import rospy
from std_msgs.msg import Float32MultiArray
import time

rospy.init_node("Sender")
pub = rospy.Publisher('msgs',Float32MultiArray,queue_size=1)
a = 0
b = 0
while True:
    data = Float32MultiArray()
    data.data = [a,b]
    pub.publish(data)
    print(data.data)
    a+=1
    b-=1
    time.sleep(0.1)

