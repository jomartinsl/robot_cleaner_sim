%% oppg 3.1

% Definerer dh parameterene for en robot

clear all
clc

syms L1 L2 L3 L4 q1 q2 q3 q4

L1 = 0.09;
L2 = 0.17325;
L3 = 0.1215;
L4 = 0.0575;

L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0);
robot = SerialLink(L) 


%% Link 1
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
robot = SerialLink(L(1)) 
robot.fkine(q1)
%%
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
robot = SerialLink(L(2)) 
robot.fkine(q2)

%%
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
robot = SerialLink(L(3)) 
robot.fkine(q3)

%%
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0);
robot = SerialLink(L(4)) 
robot.fkine(q4)

%%
q = [deg2rad(0) deg2rad(90) deg2rad(-104.4) deg2rad(-75.6)];               % Leddvariabler
dq = [-10, 0, 0, 0];                                                        %Leddhastigheter

JacobianMatrise = robot.jacobe(q);

Fart = JacobianMatrise * dq';

V = Fart(1:3)
W = Fart(4:6)
J = JacobianMatrise

robot.teach()
