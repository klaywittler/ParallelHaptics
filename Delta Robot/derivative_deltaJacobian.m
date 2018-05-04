function [dJ] = derivative_deltaJacobian(u)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    q = u(1:3);
    qd = u(4:6);
    X = u(7:9);
    dX = u(10:12);
    
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
    db = b;
    ds = s;

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
        b(:,i) = R_t(:,:,i)*[-La*sin(q(i)); 0; La*cos(q(i))];
        s(:,i) = X - R_t(:,:,i)*([R;0;0] + [La*cos(q(i)); 0; -La*sin(q(i))]);
        db(:,i) = R_t(:,:,i)*[-La*cos(q(i)); 0 ; -La*sin(q(i))]*qd(i);
        ds(:,i) = dX - b(:,i)*qd(i);
    end
    
    % Jacobian
    J = deltaJacobian([q;X]);
    dJ = s\(-ds'*J + diag([ds(:,1)'*b(:,1)+s(:,1)'*db(:,1), ds(:,2)'*b(:,2)+s(:,2)'*db(:,2), ds(:,3)'*b(:,3)+s(:,3)'*db(:,3)]));

end

