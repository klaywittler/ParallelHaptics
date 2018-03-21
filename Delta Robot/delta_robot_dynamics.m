close all; clear all; %clc;

Ra = 0.5; %distance to motor on base plate
Rb = 0.25; %distance to forearme connection on traveling plate
R = Ra - Rb; %difference between base plate and traveling plae
La = 1; %length of the arm
Lb = (La + R)*sqrt(2); %length of the forearm
n_total = 10; %number of steps
dt = 0.1;
g = 9.81; %gravity

mn = 1; %mass of the traveling plae
mab = 1; %mass of the forearm
mb = 1; %mass of the arm
mc = 1; %mass of the elbow
mnt = mn + mab/3;
Im = 1; %inertia of the motor
Ib = Im + (La^2)*(mb/3 + mc);
Ib0 = Im + (La^2)*(mb/3 + mc + (2/3)*mab);
Ibt = diag([Ib0 Ib0 Ib0]);
Aarms = diag([Ib Ib Ib]);
Gn = mn*[0; 0; -g];
Gb = zeros(3,n_total);
mbr = mb + mc;
rGb = La*(0.5*mb + mc)/(mb + mc);

phi = [0; 120*pi/180; 240*pi/180]; %rotation angle between motors

qd = zeros(3,n_total); %joint velocities
qdd= zeros(3,n_total); %joint accelerations
q = zeros(3,n_total); %joint angles
q(:,1) = [0*pi/180 0*pi/180 0*pi/180]; %initial angle

X = zeros(3,n_total); %traveling plate position
Xd = [0; 0; -0.1]; %traveling plate velocity
Xdd = zeros(3,n_total); %traveling plate acceleration

R_t = zeros(3,3,3); %motor rotation matrix
r_t = zeros(3,3,3); %rotation matrix for drawing the base plate
p = zeros(3,3,n_total); %elbow position
%kinematic chain
s = zeros(3,3,n_total);
sd = zeros(3,3,n_total);
b = zeros(3,3,n_total);
bd = zeros(3,3,n_total);
Ju = zeros(3,3,1);
v = zeros(3,3);
a = zeros(3,3);
T = zeros(3,n_total);
z = [0; 0; 0]; %zero matrix

% create rotation matrices
for i = 1:3
    R_t(:,:,i) = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
    r_t (:,:,i) = 2*[cos(phi(i) + deg2rad(60)) -sin(phi(i) + deg2rad(60)) 0; sin(phi(i) + deg2rad(60)) cos(phi(i) + deg2rad(60)) 0; 0 0 1];    
end

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
        
        v(:,i) = -R_t(:,:,i)*[La*sin(q(i,n)); 0; La*cos(q(i,n))];
    end

    % Jacobian
    J = -[s(:,1,n)'; s(:,2,n)'; s(:,3,n)']\diag([s(:,1,n)'*b(:,1,n) s(:,2,n)'*b(:,2,n) s(:,3,n)'*b(:,3,n)]);
    
    % joint velocities
    qd(:,n) = J\Xd;
    
    % create kinematic chain derivative
    for i = 1:3
       sd(:,i,n) = Xd + R_t(:,:,i)*[La*sin(q(i,n)); 0; -La*cos(q(i,n))]*qd(i,n);
       bd(:,i,n) = R_t(:,:,i)*[La*cos(q(i,n)); 0; -La*sin(q(i,n))]*qd(i,n);
       Gb = mbr*rGb*g*[cos(q(1,n)); cos(q(2,n)); cos(q(3,n))];
    end

    % update joint angle
    q(:,n+1) = q(:,n) + qd(:,n)*dt;
    
    if n > 1
        qdd(:,n) = (qd(:,n) - qd(:,n-1))/dt;
    else
        qdd(:,n) = qd(:,n)/dt;
    end
    
    d = [sd(:,1,n)'*b(:,1,n) + s(:,1,n)'*bd(:,1,n) sd(:,2,n)'*b(:,2,n) + s(:,2,n)'*bd(:,2,n) sd(:,3,n)'*b(:,3,n) + s(:,3,n)'*bd(:,3,n)];
    Xdd(:,n) = -[s(:,1,n)'; s(:,2,n)'; s(:,3,n)']\(([sd(:,1,n)'; sd(:,2,n)'; sd(:,3,n)']*J + diag(d))*qd(:,n)) + J*qdd(:,n);
    
    An = mn*(J'*J);      
       
    Ju(:,:,1) =  [v(:,1) z z];
    Ju(:,:,2) =  [z v(:,2) z];
    Ju(:,:,3) =  [z z v(:,3)];
    
    Aforearms = 0;
    Tab = 0;
    for i = 1:3
        Aforearms = Aforearms + (1/3)*mab*((J'*J) + (Ju(:,:,i)'*Ju(:,:,i)) + (Ju(:,:,i)'*J));
        a(:,i) = -R_t(:,:,i)*([La*sin(q(i,n)); 0; La*cos(q(i,n))]*qdd(i,n) + [La*cos(q(i,n)); 0; -La*sin(q(i,n))]*(q(i,n)^2));
        Tab = Tab + (1/3)*mab*(J'*(Xdd(:,n) + 0.5*a(:,i)) + Ju(:,:,i)'*(a(:,i) + 0.5*Xdd(:,n))) - 0.5*(J + Ju(:,:,i))'*mab*[0; 0; -g];
    end
    
    A = An + Aarms + Aforearms;
    A_0 = Ibt + mnt*(J'*J);
    
    T(:,n) = Aarms*qdd(:,n) + J'*Xdd(:,n) - Gb - J'*Gn + Tab ;
end

%animate_simulation(n_total,X,q)