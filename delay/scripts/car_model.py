import math
import rospy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState

class carNode():
    def __init__(self,init_time):
        super().__init__()
        self.wm = 0
        self.wz = 0
        self.vx = 0
        self.vy = 0
        self.theta1 = 4.5355
        self.theta2 = 12985
        self.theta3 = 191.17
        self.theta4 = 85.018
        self.theta5 = 696.47
        self.theta6 = 370.3704
        self.Cw = 1880.5
        self.Cwd = 1000
        self.J = 22.1185
        self.rg = 9.49
        self.m = 2.7
        self.Rw = 0.024
        self.L = 0.256
        self.prev_time = init_time
        self.alpha = 0
        self.delta = 0
        self.subtime = 0
        self.cockpit_subscribe = rospy.Subscriber("/delay/racing_cockpit/ctrl_cmd",  Vector3Stamped, self.ctrl_callback, queue_size=10)
        self.velocity_publisher = rospy.Publisher("/model/velocity",  JointState, queue_size=10)

    def update(self):
        current_time = rospy.Time.now()
        alpha = self.alpha
        delta = self.delta
        ts = current_time - self.prev_time
        ts = float(ts.to_sec()) 
        print("TS:",ts)
        self.prev_time = current_time

        A1 = -self.theta1 * self.wm
        A2 = self.theta2 * alpha
        wmd = A1 + A2
        wmd = self.wm + wmd*ts

        B1 = self.theta3 * self.L * delta * math.cos(delta) * self.vx / 2
        B2 = self.theta3 * self.L * (1 - math.cos(delta)) * self.vy / 2
        B3 = -self.L**2 *((self.theta3 *(1 + math.cos(delta)) / self.L )+ self.theta4) * self.wz / 2
        B4 = self.Rw * self.L * self.theta4 * math.sin(delta) * self.wm / (2* self.rg)
        wzd = B1 + B2 + B3 + B4
        wzd = self.wz + wzd*ts

        C1 = - self.vx * (2 * self.theta5 + self.theta6 * delta * math.sin(delta))
        C2 = self.theta6 * math.sin(delta) * self.vy
        C3 = -self.wz * self.vy
        C4 = self.L * self.theta6 * math.sin(delta)* self.wz / 2
        C5 = self.theta5 * self.Rw * (1+math.cos(delta)) * self.wm / self.rg
        vxd = C1 + C2 + C3 + C4 + C5
        vxd = self.vx + vxd*ts

        D1 = self.theta6 * delta * math.cos(delta) * self.vx
        D2 = -self.theta6 * (1 + math.cos(delta)) * self.vy
        D3 = -self.wz * self.vx
        D4 = self.L * (self.theta6 *( 1 - math.cos(delta)) - 2 * self.theta5 ) * self.wz / 2
        D5 = self.theta5 * self.Rw * math.sin(delta) * self.wm / self.rg
        vyd = D1 + D2 + D3 + D4 + D5
        vyd = self.vy + vyd*ts

        self.wm = wmd
        self.wz = wzd
        self.vx = vxd
        self.vy = vyd

        # return self.vx,ts
        self.publish_velocity()
    
    def publish_velocity(self):
        state = JointState()
        state.header.stamp = self.subtime
        state.velocity = self.vx
        self.velocity_publisher.publish(state)

    def ctrl_callback(self,data):
        self.delta = data.vector.x
        self.alpha = 0.0
        if data.vector.z > 0.5:
            self.alpha = data.vector.y
        if data.vector.z < -0.5:
            self.alpha = -1.0 * data.vector.y
        self.subtime = data.header.stamp

if __name__ == '__main__':
    rospy.init_node('carnode')
    car = carNode(rospy.Time.now())
    #rospy.on_shutdown(car.shutdown)

    try:
        car.update()
        # rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')
