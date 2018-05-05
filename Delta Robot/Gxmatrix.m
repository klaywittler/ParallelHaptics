function [ Gx,Gy,Gz ] = Gxmatrix( u )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    q = u(1:3);
    X = u(4:6);

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
    g = 9.81; %gravitation acceleration
    mnt = mn + 3*(1-r)*mfb + mPay;
    mbr = mb + mc + r*mfb;
    rGb = La*(0.5*mb + mc + r*mfb)/mbr;

    Tgn = J'*mnt*[0; 0; -g];
    Gb = mbr*g;
    Tgb = rGb*Gb*[q(1); q(2); q(3)];

    Gq = Tgn + Tgb;
    G = (J^-1)'*Gq;
    Gx = G(1);
    Gy = G(2);
    Gz = G(3);
end

