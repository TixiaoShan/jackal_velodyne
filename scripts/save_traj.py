#!/usr/bin/env python
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

if __name__ == "__main__":
    rospy.init_node('save_traj')
    get_link_state_srv_name = '/gazebo/get_link_state'
    rospy.wait_for_service(get_link_state_srv_name)
    get_link_state_srv = rospy.ServiceProxy(get_link_state_srv_name, GetLinkState)

    states = []

    def save_traj():
        np.savetxt('/tmp/gazebo_traj.csv', np.array(states)[1:])

    rospy.on_shutdown(save_traj)

    rate = rospy.Rate(10)
    req = GetLinkStateRequest()
    req.link_name = 'jackal::base_link'
    while not rospy.is_shutdown():
        try:
            resp = get_link_state_srv(req)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            break

        t = rospy.Time.now().to_sec()
        x = resp.link_state.pose.position.x
        y = resp.link_state.pose.position.y
        z = resp.link_state.pose.position.z
        q = (resp.link_state.pose.orientation.x, resp.link_state.pose.orientation.y, resp.link_state.pose.orientation.z,
             resp.link_state.pose.orientation.w)
        r, p, y = euler_from_quaternion(q)
        states.append((t, x, y, z, r, p, y))
        rate.sleep()
