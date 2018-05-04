function [ C ] = Cmatrix( u )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    q = u(1:3);
    X = u(4:6);
    qd = u(7:9);
    dX = u(10:12);
    
    J = deltaJacobian([q;X]);
    dJ = derivative_deltaJacobian(u);
    
    mPay = 1; %mass of payload
    mn = 1; %mass of the traveling plae
    mfb = 1; %mass of the forearm

    r = 2/3; %amount of forearm mass at upper extreme
    
    mnt = mn + 3*(1-r)*mfb + mPay;
    
    C = J'*mnt*dJ;
    %C = (J^-1)'*Cq

end

