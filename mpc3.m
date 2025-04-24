
m = 1400;      
Iz = 1960;     
lf = 1.177;    
lr = 1.358;    
Cf = 84085;    
Cr = 87342;   
Ts = 0.1;     
vx = 10;      


A_cont = [ -(Cf + Cr) / (m * vx),      ((Cr * lr - Cf * lf) / (m * vx)) - vx;
           (Cr * lr - Cf * lf) / (Iz * vx), -(lf^2 * Cf + lr^2 * Cr) / (Iz * vx)];

B_cont = [ Cf / m;
           (lf * Cf) / Iz ];


Ad = eye(2) + A_cont * Ts;   
Bd = B_cont * Ts;


Cd = [0 1];  
Dd = 0;      


sys_d = ss(Ad, Bd, Cd, Dd, Ts);


sys_d

save('mpc_plant.mat', 'sys_d');