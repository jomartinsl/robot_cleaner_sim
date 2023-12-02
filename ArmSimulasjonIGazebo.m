%% ELE306 turtlebot lab number 2
clc; 
clear; 
close all;
import ETS3.*

% Setting up environment
setenv("ROS_DOMAIN_ID","30");
% Initializing ros node
armControllerNode = ros2node("/open_manipulator_controller");

trajPub = ros2publisher(armControllerNode, "/joint_trajectory_controller/joint_trajectory", "trajectory_msgs/JointTrajectory");
pause(3)

% Defining message for publisher
trajMsg = ros2message(trajPub);
trajMsg.joint_names = {'base_arm_joint', 'link_1_joint', 'link_2_joint', 'link_3_joint'};

%%
% Defining the robotic arm
L1 = 0.09;
L2 = 0.17325;
L3 = 0.1215;
L4 = 0.0575;



j1 = Revolute('d', L1, 'a', 0, 'alpha', pi/2, 'offset',0);
j2 = Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0);
j3 = Revolute('d', 0, 'a', L3, 'alpha', 0, 'offset', 0);
j4 = Revolute('d', 0, 'a', L4, 'alpha', 0, 'offset', 0);

robot = SerialLink([j1 j2 j3 j4],'name', 'my robot');
robot.qlim = [-3.14, +3.14; -1.57, +1.57; -1.40, +1.57; -1.57, 1.57];


robot.plot([0, 0, 0, 0])

%% 
% Make sure the gripper is open  /gripper_controller/gripper_cmd

% [client,goalMsg] = ros2actionclient(armControllerNode,"/gripper_controller/gripper_cmd","control_msgs/GripperCommand");
% 
% status = waitForServer(client)
% goalMsg.command.position= 0.0;
% goalMsg.command.max_effort = 0.0;
% 
% 
% %callbackOpts = ros2ActionSendGoalOptions(FeedbackFcn=@helperFeedbackCallback,ResultFcn=@helperResultCallback);
% goalHandle = sendGoal(client,goalMsg); %, callbackOpts);
% exStatus = getStatus(client,goalHandle)
% 
% resultMsg = getResult(goalHandle);
% rosShowDetails(resultMsg)

%% 
% Gripper subscriber
% Make sure the gripper is open

gripperPub = ros2publisher(armControllerNode, "/gripper_control/commands", "std_msgs/Float64MultiArray");
gripperMsg = ros2message(gripperPub);
gripperMsg.data = [0.0];
send(gripperPub,gripperMsg)


%%
% Go first to "zero" point 
point1 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point1.positions =  [0 deg2rad(0) deg2rad(0) deg2rad(0)];
point1.velocities = [0, 0, 0,0];
point1.accelerations = [0, 0, 0, 0];
point1.effort = [0, 0, 0, 0];
point1.time_from_start.sec = int32(2);

% Check in the plot that it looks okay

robot.plot(point1.positions);
pause(5)

%%

T_robot_goal_1 = SE3(0.25, 0 , -0.01) * SE3.rpy(0,0,90, 'deg');
q1 = robot.ikcon(T_robot_goal_1, point1.positions) % this takes into account joint limits and the inital position

% Check in the plot that it looks okay

robot.plot(q1)

%%
point2 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point2.positions =  ([0 deg2rad(50) deg2rad(-45) 0]);
point2.velocities = [0, 0, 0,0];
point2.accelerations = [0, 0, 0, 0];
point2.effort = [0, 0, 0, 0];
point2.time_from_start.sec = int32(5);
trajMsg.points = [point1, point2];
send(trajPub, trajMsg)

pause(2)

%% 
% Grip the cup  /gripper_topic
gripperMsg.data = 0.016;
send(gripperPub,gripperMsg)

%% 
% Take the cup "in"

T_robot_goal_2 = SE3(0.25, 0 , -0.01) * SE3.rpy(0,0,90, 'deg');
q2 = robot.ikcon(T_robot_goal_2, point2.positions) % this takes into account joint limits and the inital position

% Check in the plot that it looks okay

robot.plot(q2);

%%
point3 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point3.positions =  [0 deg2rad(4.8) deg2rad(-30.6) 0];
point3.velocities = [0, 0, 0,0];
point3.accelerations = [0, 0, 0, 0];
point3.effort = [0, 0, 0, 0];
point3.time_from_start.sec = int32(5);
trajMsg.points = point3;
send(trajPub, trajMsg);

pause(2)

point4 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point4.positions =  ([0 deg2rad(44) deg2rad(-30.6) 0]);
point4.velocities = [0, 0, 0,0];
point4.accelerations = [0, 0, 0, 0];
point4.effort = [0, 0, 0, 0];
point4.time_from_start.sec = int32(5);
trajMsg.points = point4;
send(trajPub, trajMsg);

pause(5)


point5 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point5.positions =  ([0 deg2rad(39) deg2rad(113.4) deg2rad(-20)]);
point5.velocities = [0, 0, 0,0];
point5.accelerations = [0, 0, 0, 0];
point5.effort = [0, 0, 0, 0];
point5.time_from_start.sec = int32(5);
trajMsg.points = point5;
send(trajPub, trajMsg);

pause(6)

point6 = ros2message('trajectory_msgs/JointTrajectoryPoint');
point6.positions =  ([0 deg2rad(30) deg2rad(60) deg2rad(0)]);
point6.velocities = [0, 0, 0,0];
point6.accelerations = [0, 0, 0, 0];
point6.effort = [0, 0, 0, 0];
point6.time_from_start.sec = int32(5);
trajMsg.points = point6;
send(trajPub, trajMsg);
%%
