function [A] = delta_nonlinearTerm(u)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    Ig = u(1:3,:);
    M = u(4:6,:);
    J2 = u(7:9,:);
    dJ1 = u(10:12,:);
    
    A = J2'*M + Ig*dJ1;

end

