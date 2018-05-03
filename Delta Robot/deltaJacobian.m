function [J] = deltaJacobian(u)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here  
    q = u(1:3);
    X = u(4:6);

    Ra = 0.5; %distance to motor on base plate
    Rb = 0.25; %distance to forearme connection on traveling plate
    R = Ra - Rb; %difference between base plate and traveling plae
    La = 1; %length of the arm
    Lb = (La + R)*sqrt(2); %length of the forearm
    
    phi = [0; 120*pi/180; 240*pi/180]; %rotation angle between motors
    R_t = zeros(3,3,3); %motor rotation matrix
    
    p = zeros(3,3); %elbow position
    
    s = zeros(3,3); 
    b = zeros(3,3); 

    for i = 1:3
        R_t(:,:,i) = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
    end
    
    for i = 1:3
        p(:,i) = [cos(phi(i))*(La*cos(q(i)) + R); -(R + La*cos(q(i)))*sin(phi(i)); -La*sin(q(i))];
    end

    % find traveling position through elbow workspace intersection
    %X = interx(p(:,1),p(:,2),p(:,3),Lb,Lb,Lb,0);

    % create kinematic chain 
    for i = 1:3
%         b(:,i) = R_t(:,:,i)*[La*cos(q(i)); 0; -La*sin(q(i))];
%         s(:,i) = X - (R_t(:,:,i)*[R;0;0] + b(:,i));
        b(:,i) = R_t(:,:,i)*[La*sin(q(i)); 0; -La*cos(q(i))];
        s(:,i) = X - R_t(:,:,i)*([R;0;0] + [La*cos(q(i)); 0; -La*sin(q(i))]);
    end

    % Jacobian
    J = -[s(:,1)'; s(:,2)'; s(:,3)']\diag([s(:,1)'*b(:,1) s(:,2)'*b(:,2) s(:,3)'*b(:,3)]);
end

