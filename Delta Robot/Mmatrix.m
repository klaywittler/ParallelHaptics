function [ M ] = Mmatrix( u )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes hereb
    
    J = deltaJacobian(u);
    
    Ra = 0.5; %distance to motor on base plate
    Rb = 0.25; %distance to forearme connection on traveling plate
    R = Ra - Rb; %difference between base plate and traveling plae
    La = 1; %length of the arm
    Lb = (La + R)*sqrt(2); %length of the forearm

    mPay = 1; %mass of payload
    mn = 1; %mass of the traveling plae
    mfb = 1; %mass of the forearm
    mb = 1; %mass of the arm
    mc = 1; %mass of the elbow
    r = 2/3; %amount of forearm mass at upper extreme
    k = 1; %gear ratio
    mnt = mn + 3*(1-r)*mfb + mPay;
    Im = 1; %inertia of the motor
    Ibc = (La^2)*(mb/3 + mc + r*mfb);
    Ibi = Im*k*r^2 + Ibc;
    Ib = diag([Ibi Ibi Ibi]);
    
    Aq = Ib + mnt*(J'*J);
    M = (J^-1)'*Aq*(J^-1);
end

