close all; clear all; %clc;

Ra = 0.5; %distance to motor on base plate
Rb = 0.25; %distance to forearme connection on traveling plate
R = Ra - Rb; %difference between base plate and traveling plae
La = 1; %length of the arm
Lb = (La + R)*sqrt(2); %length of the forearm
n_total = 10; %number of steps
dt = 0.1;

phi = [0; 120*pi/180; 240*pi/180]; %rotation angle between motors

qd = zeros(3,n_total); %joint velocities
q = zeros(3,n_total); %joint angles
q(:,1) = [0*pi/180 0*pi/180 0*pi/180]; %initial angle

X = zeros(3,n_total); %traveling plate position
Xd = [0; 0; -0.1]; %traveling plate velocity

R_t = zeros(3,3,3); %motor rotation matrix
r_t = zeros(3,3,3); %rotation matrix for drawing the base plate
p = zeros(3,3,n_total); %elbow position
%kinematic chain
s = zeros(3,3,n_total); 
b = zeros(3,3,n_total); 

% create rotation matrices
for i = 1:3
    R_t(:,:,i) = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
    r_t (:,:,i) = 2*[cos(phi(i) + deg2rad(60)) -sin(phi(i) + deg2rad(60)) 0; sin(phi(i) + deg2rad(60)) cos(phi(i) + deg2rad(60)) 0; 0 0 1];    
end

% 
for n = 1:n_total
    
    % elbow positions
    for i = 1:3
        p(:,i,n) = [cos(phi(i))*(La*cos(q(i,n)) + R); -(R + La*cos(q(i,n)))*sin(phi(i)); -La*sin(q(i,n))];
    end

    % find traveling position through elbow workspace intersection
    X(:,n) = interx(p(:,1,n),p(:,2,n),p(:,3,n),Lb,Lb,Lb,0);

    % create kinematic chain 
    for i = 1:3
        b(:,i,n) = R_t(:,:,i)*[La*cos(q(i,n)); 0; -La*sin(q(i,n))];
        s(:,i,n) = X(:,n) - (R_t(:,:,i)*[R;0;0] + b(:,i,n));
    end

    % Jacobian
    J = -[s(:,1,n)'; s(:,2,n)'; s(:,3,n)']\diag([s(:,1,n)'*b(:,1,n) s(:,2,n)'*b(:,2,n) s(:,3,n)'*b(:,3,n)]);

    % joint velocities
    qd(:,n) = J\Xd;

    % update joint angle
    q(:,n+1) = q(:,n) + qd(:,n)*dt;

end

animate_simulation(n_total,X,q)