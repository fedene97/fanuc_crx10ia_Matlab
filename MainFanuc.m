clc
clear
close all

FANUC = importrobot('./crx10ial.urdf');
FANUC.DataFormat='row';
ik = robotics.InverseKinematics('RigidBodyTree', FANUC);
weights = [1, 1, 1, 1, 1, 1];
initialguess=[0 0 0 0 0 0];

X_i=[0.25 -0.66 0.83 0 pi/2 0]';
eul_i=X_i(4:6);
R_i=eul2rotm(eul_i','ZYZ');
X_model_i=X_i;
X_model_i(4:6)=rotm2eul(R_i,'ZYZ');
pose=[eul2rotm(X_model_i(4:6)','ZYZ') X_model_i(1:3); 0 0 0 1];
Q = ik('ee_link',pose,weights,initialguess);

show(FANUC,Q, 'Frames','on');
