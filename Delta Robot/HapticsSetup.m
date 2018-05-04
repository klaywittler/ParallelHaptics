%% Haptics Setup

    Ra = 0.5; %distance to motor on base plate
    Rb = 0.25; %distance to forearme connection on traveling plate
    R = Ra - Rb; %difference between base plate and traveling plae
    La = 1; %length of the arm
    Lb = (La + R)*sqrt(2); %length of the forearm
    
    phi = [0; 120*pi/180; 240*pi/180]; %rotation angle between motors
    R_t = zeros(3,3,3); %motor rotation matrix
    
    p = zeros(3,3); %elbow position

    for i = 1:3
        R_t(:,:,i) = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
    end

% Ground Contact Model
K_e = 10000000;    % Object Stiffness, [N/m] or [Nm/rad]
B_e = 50;    % Object Damping,   [Ns/m] or [Nms/rad]

% Delta Robot
m = 1; %mass of the traveling plate
Im = [1 0 0; 0 1 0; 0 0 1]; %moment of intertia in the base frame
M = m*eye(3)*Im;
Ig = [1 0 0; 0 1 0; 0 0 1];

q0 = [0;0;0];

% Simulation
for i = 1:3
        p(:,i) = [cos(phi(i))*(La*cos(q0(i)) + R); -(R + La*cos(q0(i)))*sin(phi(i)); -La*sin(q0(i))];
end

    % find traveling position through elbow workspace intersection
x0 = interx(p(:,1),p(:,2),p(:,3),Lb,Lb,Lb,0);

xd0 = [0;0;0];
qd0 = inv(deltaJacobian([q0;x0]))*xd0;