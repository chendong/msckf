function quat_new=propagateQuaternionOneStep(quat_old,w_new,w_old,dt)
% using q = [0;0;0;1] convention

omega_old = omegaMat(w_old);
omega_new = omegaMat(w_new);
omega_mean = omegaMat((w_new+w_old)/2);


div =1;
mat_exp = eye(4);
omega_mean = omega_mean*0.5*dt;
omega_mean_integarate = omega_mean;

for i=1:4
    div = div*i;
    mat_exp = mat_exp+omega_mean_integarate/div;
    omega_mean_integarate = omega_mean_integarate*omega_mean;
end
%quat_int is a 4x4 matrix
quat_int = mat_exp+1.0/48.0*(omega_new*omega_old - omega_old*omega_new)*dt*dt;
quat_int = quat_int/norm(quat_int);

quat_new = quat_int*quat_old; 
% 
%  psi = w_new*dt;
%  quat_new = quat_old + 0.5 * omegaMat(psi) * quat_old;
end
