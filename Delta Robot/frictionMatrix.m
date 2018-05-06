function [friction] = frictionMatrix(u)
q = u(1:3);
x = u(4:6);
dq = u(7:9);
dx = u(10:12);

Kq = 10*eye(3);
J = deltaJacobian([q;x]);
Kx = (J')^(-1)*Kq*(J)^(-1);
friction = Kx*dx;
end