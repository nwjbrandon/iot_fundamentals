#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetPhysicsProperties
from std_msgs.msg import Float64
import geometry_msgs
import gazebo_msgs

class GazeboConnection():
    
    def __init__(self):
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_gravity_proxy = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        self.set_gravity()
        self.pause_sim()

    def set_gravity(self):
        time_step = Float64(0.001)
        max_update_rate = Float64(1000.0)
        gravity = geometry_msgs.msg.Vector3()
        gravity.x = 0.0
        gravity.y = 0.0
        gravity.z = -9.8
        ode_config = gazebo_msgs.msg.ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = 50
        ode_config.sor_pgs_w = 1.3
        ode_config.sor_pgs_rms_error_tol = 0.0
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 0.0
        ode_config.cfm = 0.0
        ode_config.erp = 0.2
        ode_config.max_contacts = 20
        self.set_gravity_proxy(time_step.data, max_update_rate.data, gravity, ode_config) 

    def pause_sim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException, e:
            print ("/gazebo/pause_physics service call failed")
        
    def unpause_sim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException, e:
            print ("/gazebo/unpause_physics service call failed")
        
    def reset_sim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_simulation_proxy()
            self.reset_world_proxy()
            self.set_gravity()
        except rospy.ServiceException, e:
            print ("/gazebo/reset_simulation service call failed")