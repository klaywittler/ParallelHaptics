function [A] = delta_massMatrix(u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    Ig = u(1:3,:);
    J2 = u(4:6,:);
    
    A = -J2*(Ig^-1);

end

