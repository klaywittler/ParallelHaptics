%% Human Arm Kinematics
% Augustus Ellis, Klayton Whittler
clear all; close all;
%% Human Arm Parameters%% Human Arm DH Parameters
deg2rad  = pi/180;
in2m = 0.0254;
DOF_HA   = 7;
L_p   = 4*in2m;      % [m]
L_f   = 12*in2m;     % [m]
L_u   = 12*in2m;     % [m]

m_p   = 1;           % [kg]
m_f   = 1;           % [kg]
m_u   = 1;           % [kg]

Icm_p   = eye(3);    % [m^4]
Icm_f   = eye(3);    % [m^4]
Icm_u   = eye(3);    % [m^4]

cm_p = [0;L_p/2;0];  % [m]
cm_f = [0;L_f/2;0];  % [m]
cm_u = [0;L_u/2;0];  % [m]


%% Build Robot Model from selected Coordinate Frames and Geometric Parameters
humanArm.gravity = [0;0;-9.81];
humanArm.NB = 3;
humanArm.parent = 0:1:humanArm.NB;
humanArm.jtype{1} = 'R';
humanArm.jtype{2} = 'R';
humanArm.jtype{3} = 'R';
humanArm.Xtree{1} = plux(rx(-pi/2), [0;0;0]);
humanArm.Xtree{2} = plux(eye(3), [0;L_u;0]);
humanArm.Xtree{3} = plux(eye(3), [0;L_f;0]);
humanArm.I{1} = mcI(m_u, cm_u, Icm_u);
humanArm.I{2} = mcI(m_f, cm_f, Icm_f);
humanArm.I{3} = mcI(m_p, cm_p, Icm_p);

%% Forward Kinematics

%% Dynamics
% Forward (RNEA)
q_trial = linspace(0, pi/2, 1000).*ones(3, 1000);
qd_trial = 1*ones(3, 1000);
qdd_trial = 1.*ones(3, 1000);
fext_trial = num2cell(0.*ones(7, 1000));
tau = zeros(3, 1000);
for i = 1:size(q_trial,2)
    tau(:,i) = ID(humanArm, q_trial(:,i), qd_trial(:,i), qdd_trial(:,i));
end
% Inverse (ABA)
for i = 1:length(tau)
[qdd_out(:,i), tau_out(:,i)] = HD( humanArm, ones(1,7), q_trial(:,i), qd_trial(:,i), qdd_trial(:,i), tau(:,i));
end

% Overall Gain Should be 0 for forward-inverse dynamics
plot(qdd_trial' - qdd_out');
xlabel('Time, normalized');
ylabel('Joint Acceleration Error');