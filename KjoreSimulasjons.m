%% ELE306 turtlebot lab number 1


% Setting up environment
setenv("ROS_DOMAIN_ID","30");
% Initializing ros node
braitenberControllerNode = ros2node("/braitenberg_controller");

% Creating subscriber to laser scan and publisher to cmd velocity
pause(4)
cmdvalPub = ros2publisher(braitenberControllerNode, "/cmd_vel", "geometry_msgs/Twist");
pause(4)

% Defining message for publisher
cmdvelMsg = ros2message(cmdvalPub);

while(true)
    cmdvelMsg.linear.x = 0.7;
    cmdvelMsg.angular.z = 0.0;          %Kjøre fram
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.0;
    cmdvelMsg.angular.z = 0.35;         %rotere
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.2;
    cmdvelMsg.angular.z = 0.0;          %Kjøre litt fram
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.0;
    cmdvelMsg.angular.z = 0.35;         %Rotere
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.7;
    cmdvelMsg.angular.z = 0.0;          %Kjøre langt frem igjen
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.0;
    cmdvelMsg.angular.z = -0.35;         %Rotere
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.2;
    cmdvelMsg.angular.z = 0.0;          %Kjøre litt fram
    send(cmdvalPub, cmdvelMsg)
    pause(5)
    cmdvelMsg.linear.x = 0.0;
    cmdvelMsg.angular.z = -0.35;         %Rotere
    send(cmdvalPub, cmdvelMsg)
    pause(5)
end













