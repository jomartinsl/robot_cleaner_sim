
clear all;

% Definerer robotens lenker med DH-parametere
L1 = 0.09;
L2 = 0.17325;
L3 = 0.1215;
L4 = 0.0575;

L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0);

% Oppretter robotmodellen
robot = SerialLink(L) 

% Definerer fire posisjoner for end-effektoren
startPos = robot.fkine([0 deg2rad(90) deg2rad(-104.4) deg2rad(-104.4)])
  pos3 = robot.fkine([0 deg2rad(50) deg2rad(-45) 0]);
  pos4 = robot.fkine([0 deg2rad(4.8) deg2rad(-30.6) 0]);
  pos5 = robot.fkine([0 deg2rad(44) deg2rad(-30.6) 0]);
  pos6 = robot.fkine([0 deg2rad(33) deg2rad(113.4) 0]);
  pos7 = robot.fkine([0 deg2rad(33) deg2rad(113.4) 0]);
  endPos = robot.fkine([0 deg2rad(33) deg2rad(113.4) deg2rad(64)]);




%Beregner invers kinematikk for hver av posisjonene
qStart = robot.ikcon(startPos, [0 deg2rad(90) deg2rad(-104.4) deg2rad(-104.4)]);        %bruker ickon på første siden det er standard og må ha de riktinge joint vinklene
q3 = robot.ikine(pos3, 'mask', [1 1 1 0 0 0]);
q4 = robot.ikine(pos4, 'mask', [1 1 1 0 0 0]);
q5 = robot.ikine(pos5, 'mask', [1 1 1 0 0 0]);
q6 = robot.ikine(pos6, 'mask', [1 1 1 0 0 0]);
q7 = robot.ikine(pos7, 'mask', [1 1 1 0 0 0]);
qend = robot.ikine(endPos, 'mask', [1 1 1 0 0 0]);


% Genererer baner mellom hver av posisjonene
steps = 25;  % Antall steg mellom hver posisjon
s = linspace(0, 1, steps);
path = [jtraj(qStart, q3, steps); 
        %jtraj(q2, q3, steps);
        jtraj(q3, q4, steps);
        jtraj(q4,q5,steps);
        jtraj(q5,q6,steps);
        jtraj(q6,q7,steps);
        jtraj(q7,qend,steps)
        ];
    

% Animerer bevegelsen
for i=1:size(path, 1)
    robot.plot(path(i,:));      % Tegner roboten ved hver posisjon langs banen
    drawnow
end