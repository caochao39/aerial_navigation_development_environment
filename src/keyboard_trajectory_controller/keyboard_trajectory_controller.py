#!/usr/bin/env python
 
import rospy
import pygame
from geometry_msgs.msg import TwistStamped
 
pygame.init()
screen = pygame.display.set_mode((400, 300))
 
rospy.init_node('twist_publisher')
 
# Create a publisher object
pub = rospy.Publisher('/attitude_control', TwistStamped, queue_size=10)
 
# Set the initial message values
twist_stamped_msg = TwistStamped()
twist_stamped_msg.header.frame_id = "vehicle"
twist_stamped_msg.twist.linear.x = 0.0
twist_stamped_msg.twist.linear.y = 0.0
twist_stamped_msg.twist.linear.z = 0.0
twist_stamped_msg.twist.angular.x = 0.0
twist_stamped_msg.twist.angular.y = 0.0
twist_stamped_msg.twist.angular.z = 0.0
 
while not rospy.is_shutdown():
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.angular.z = 1.0
            if event.key == pygame.K_e:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.angular.z = -1.0
 
            if event.key == pygame.K_w:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.y = 0.5
 
            if event.key == pygame.K_s:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.y = -0.5
 
            if event.key == pygame.K_a:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.x = -0.5
 
            if event.key == pygame.K_d:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.x = 0.5
 
            if event.key == pygame.K_z:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.z = 0.5
 
            if event.key == pygame.K_x:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.z = -0.5
 
 
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_q:
                # Set angular z back to 0.0 when q key is released
                twist_stamped_msg.twist.angular.z = 0.0
            if event.key == pygame.K_e:
                # Set angular z back to 0.0 when q key is released
                twist_stamped_msg.twist.angular.z = 0.0
            if event.key == pygame.K_w:
                # Set angular z back to 0.0 when q key is released
                twist_stamped_msg.twist.linear.y = 0.0
            if event.key == pygame.K_s:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.y = 0.0
 
            if event.key == pygame.K_a:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.x = 0.0
 
            if event.key == pygame.K_d:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.x = 0.0
 
            if event.key == pygame.K_z:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.z = 0.0
 
            if event.key == pygame.K_x:
                # Set angular z to 2.0 when q key is pressed
                twist_stamped_msg.twist.linear.z = 0.0
         
            elif event.key == pygame.K_ESCAPE:
                pygame.quit()
                exit()
 
    # Publish the message if the angular.z value is not 0.0
    if twist_stamped_msg.twist.angular.z != 0.0:
        twist_stamped_msg.header.stamp = rospy.Time.now()
        pub.publish(twist_stamped_msg)
 
    if twist_stamped_msg.twist.linear.x != 0.0:
        twist_stamped_msg.header.stamp = rospy.Time.now()
        pub.publish(twist_stamped_msg)
 
    if twist_stamped_msg.twist.linear.y != 0.0:
        twist_stamped_msg.header.stamp = rospy.Time.now()
        pub.publish(twist_stamped_msg)
 
    if twist_stamped_msg.twist.linear.z != 0.0:
        twist_stamped_msg.header.stamp = rospy.Time.now()
        pub.publish(twist_stamped_msg)
 
    # Wait for a small amount of time to avoid using too much CPU
    pygame.time.wait(10)