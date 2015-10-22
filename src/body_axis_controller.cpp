/****************************************************************************
 *
 *   Copyright (c) 2015 Crossline Drone Project Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name CLDrone or Crossline Drone nor the names of its c
 *    ontributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "body_axis_controller.h"
#include "tf/tf.h"
#include <math.h>


BodyAxisController::BodyAxisController()
{
    localPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    localPositionSubscriber = nodeHandle.subscribe("/mavros/local_position/local", 10, &BodyAxisController::localPositionReceived,this);
    bodyAxisPositionSubscriber = nodeHandle.subscribe("/CLDrone/body_axis_position/local",10, &BodyAxisController::bodyAxisPositionReceived,this);

}

void BodyAxisController::localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   currentPosition = *msg;
}

void BodyAxisController::bodyAxisPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
	ROS_INFO_STREAM("body axis position" << msg);

	geometry_msgs::PoseStamped bodyAxisPosition = *msg;

	geometry_msgs::PoseStamped localPositionTransfered = transferBodyAxisToLocal(bodyAxisPosition,currentPosition);

	localPositionTransfered.header.stamp = ros::Time::now();

	localPositionPublisher.publish(localPositionTransfered);
}

geometry_msgs::PoseStamped BodyAxisController::transferBodyAxisToLocal(geometry_msgs::PoseStamped bodyAxisPosition,geometry_msgs::PoseStamped currentPosition)
{
	geometry_msgs::PoseStamped transferedLocalPosition;

	tf::Pose currentPose;
	tf::poseMsgToTF(currentPosition.pose,currentPose);
	double yawAngle = tf::getYaw(currentPose.getRotation());
    
    /*
    tf::Pose bodyAxisPose;
    tf::poseMsgToTF(bodyAxisPosition.pose,bodyAxisPose);
    double targetYawAngle = tf::getYaw(bodyAxisPose.getRotation());
    ROS_INFO_STREAM("target yaw angel" << targetYawAngle*180/3.1415926);
    double transferedYawAngle = yawAngle + targetYawAngle;
    transferedLocalPosition.pose.orientation = tf::createQuaternionMsgFromYaw(transferedYawAngle);
    */

	transferedLocalPosition.pose.position.x = currentPosition.pose.position.x + bodyAxisPosition.pose.position.x * cos(yawAngle) - 
    										bodyAxisPosition.pose.position.y * sin(yawAngle);
    transferedLocalPosition.pose.position.y = currentPosition.pose.position.y + bodyAxisPosition.pose.position.x * sin(yawAngle) + 
    										bodyAxisPosition.pose.position.y * cos(yawAngle);
    transferedLocalPosition.pose.position.z = currentPosition.pose.position.z + bodyAxisPosition.pose.position.z;
    

    return transferedLocalPosition;

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "body_axis_controller");
    
    BodyAxisController controller;

    ros::spin();

    return 0;

}