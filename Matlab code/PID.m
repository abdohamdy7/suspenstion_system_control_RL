%% PID controller design

m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

% m1 = 5333;% Sprung Mass (kg)
% m2 = 906.5;% Unsprung Mass (kg)
% k1 = 430000;% Suspension Stiffness (N/m) ks
% k2 = 2440000;% Wheel stiffness (N/m) kt
% b1 = 20000;% Suspension Inherent Damping coefficient (sec/m) cs
% b2 = 40000;% Wheel Inhenrent Damping coefficient (sec/m) ct

nump=[(m1+m2) b2 k2];
denp=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G1=tf(nump,denp);

num1=[-(m1*b2) -(m1*k2) 0 0];
den1=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G2=tf(num1,den1);

numf=num1;
denf=nump;
F=tf(numf,denf);

% Passive system
t=0:0.05:5;
% step(0.1*G1,t)


% assigning the PID controller
Kd = 208025;
Kp = 832100;
Ki = 624075;
C = pid(Kp,Ki,Kd);

sys_cl=F*feedback(G1,C);
% 
% hold on
% step(0.1*sys_cl,t)
% title('Response to a 0.1-m Step under PID Control')
% grid on
% legend("Passive", "PID")