function [y] = getCollision(u)
% checkCollision determines whether the end effector of the haptic
% interface has collided with the virtual object and returns the collision
% distance
% P = [x; y; z; theta_x, theta_y; theta_z]
%                 ______
%                      |
%      ________________|_____ ___  Surface
%                      |       |
%                  EE |-|     _|_ x
% 
% The virutal object is located a distance 0.5m from the robot base and is
% perpendicular to the z direction. Thus,
%           x = Zobj - P(3)
P = u(1:3);
V = u(4:6);
Zobj = -1.5;
if P(3) <= Zobj
    x = -(P(3) - Zobj);
    x_dot = -V(3)*(V(3)<=0);
else
    x = 0;
    x_dot = 0;
end
y = [x;x_dot];
end

