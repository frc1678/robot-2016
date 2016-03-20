pkg load control;
stall_current = 134;
stall_torque = 0.71;
free_speed = 18730;
K_t = stall_torque/stall_current;
K_v = 12/free_speed;
R = 12/stall_current;
G = 1/40;
r = 1.43/2;
mass = 18;
kF = ((K_t*K_v)/(G*G*R*r*r*mass));
kM = ((K_t)/(G*R*r*mass));
A_c = [0 1
       0 -kF];
B_c = [0
       kM];
C_c = [1 0];
p_L = [-2 -0.01];
p_K = [-6+3i -6-3i];
L = place(A_c', C_c', p_L)'
K = place(A_c, B_c, p_K)
t = 0:.01:5;
u = zeros(size(t));
x0 = [1
      0];
lsim(ss(A_c - B_c*K, B_c), u, t, x0, 'b')
lsim(ss(A_c - L*C_c, L), u, t, x0, 'r')

pause;
