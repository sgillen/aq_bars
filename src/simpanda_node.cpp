/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <aqbar/falconPos.h>
#include <aqbar/falconForces.h>


geometry_msgs::Pose g_pose_goal;
aqbar::falconForces forces;
geometry_msgs::Pose current_pose;

double force_k = 10.0; //constant to multiply forces sent to the falcon by


void posCallback(const aqbar::falconPosConstPtr& msg)
{

    g_pose_goal.position.x = msg->X + .3;
    g_pose_goal.position.y = msg->Y + .3;
	g_pose_goal.position.z = msg->Z + .3;
		
	std::cout << "pose received: " << msg->X << ", " << msg->Y << ", " << msg->Z << std::endl;

	
	forces.X = (g_pose_goal.orientation.x - current_pose.orientation.x)*force_k;
	forces.Y = (g_pose_goal.orientation.y - current_pose.orientation.y)*force_k;
	forces.Z = (g_pose_goal.orientation.z - current_pose.orientation.z)*force_k;
	//	force_pub.publish(forces);


}




// TODO make this in a class, possibly thread the whole thing

// class SimPandaNode(){
//  public:
// }  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
	move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Planning to a Posel goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  g_pose_goal.orientation.x = 1.0;
  g_pose_goal.position.x = 0.28;
  g_pose_goal.position.y = -0.2;
  g_pose_goal.position.z = 0.5;
  move_group.setPoseTarget(g_pose_goal);
  
  ros::Subscriber sub = node_handle.subscribe("/falconPos", 10, &posCallback, ros::TransportHints().tcpNoDelay());
  ros::Publisher force_pub = node_handle.advertise<aqbar::falconForces>("/falconForce",10);
  
  //This can slow down all motion if we decide we want that.
  //  move_group.setMaxVelocityScalingFactor(0.1);

	
  while(node_handle.ok()){
	// Now, we call the planner to compute the plan and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	//moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	//bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	
	move_group.setPoseTarget(g_pose_goal);
	
	//It's unclear if we actually need to call this or not
	move_group.move();
	
	
	current_pose = move_group.getCurrentPose().pose;

	
	// we may actually want to compute this in the falcon driver...
		
  }
  ros::shutdown();
  return 0;
}
