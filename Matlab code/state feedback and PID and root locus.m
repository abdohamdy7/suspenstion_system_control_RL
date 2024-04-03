%% system modelling
M1 = 2500;
M2 = 320;
K1 = 80000;
K2 = 500000;
b1 = 350;
b2 = 15020;

s = tf('s');
G1 = ((M1+M2)*s^2+b2*s+K2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));
G2 = (-M1*b2*s^3-M1*K2*s^2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));

%% system analysis

M1 = 2500;
M2 = 320;
K1 = 80000;
K2 = 500000;
b1 = 350;
b2 = 15020;

%step with unit force input

s = tf('s');
G1 = ((M1+M2)*s^2+b2*s+K2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));

step(G1)

%%
%step with unit road disturbance input
G2 = (-M1*b2*s^3-M1*K2*s^2)/((M1*s^2+b1*s+K1)*(M2*s^2+(b1+b2)*s+(K1+K2))-(b1*s+K1)*(b1*s+K1));

step(0.1*G2)

%% PID controller design

m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

nump=[(m1+m2) b2 k2];
denp=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G1=tf(nump,denp);

num1=[-(m1*b2) -(m1*k2) 0 0];
den1=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G2=tf(num1,den1);

numf=num1;
denf=nump;
F=tf(numf,denf);

% assigning the PID controller
Kd = 208025;
Kp = 832100;
Ki = 624075;
C = pid(Kp,Ki,Kd);

sys_cl=F*feedback(G1,C);

t=0:0.05:5;
step(0.1*sys_cl,t)
title('Response to a 0.1-m Step under PID Control')

%% root locus PID design
z1=1;
z2=3;
p1=0;
s = tf('s');
C = ((s+z1)*(s+z2))/(s+p1);
rlocus(C*G1)
title('root locus with PID controller')

[k,poles]=rlocfind(C*G1)

%%
Kd=2*Kd;
Kp=2*Kp;
Ki=2*Ki;
C=pid(Kp,Ki,Kd);
sys_cl=F*feedback(G1,C);
step(0.1*sys_cl,t)
title('Response to a 0.1-m Step w/ High-Gain PID')
axis([0 5 -.01 .01])

%% Root locus controller design

m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

nump=[(m1+m2) b2 k2];
denp=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G1=tf(nump,denp);

num1=[-(m1*b2) -(m1*k2) 0 0];
den1=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G2=tf(num1,den1);

numf=num1;
denf=nump;
F=tf(numf,denf);

R = roots(denp)



%%

rlocus(G1)


% maximum overshoot of 0.05 --> we get the needed zeta
z=-log(0.05)/sqrt(pi^2+(log(0.05)^2))
sgrid(z,0)

%%
z1=3+3.5i;
z2=3-3.5i;
p1=30;
p2=60;
numc=conv([1 z1],[1 z2]);
denc=conv([1 p1],[1 p2]);
C=tf(numc,denc);

rlocus(C*G1)

%%
axis([-40 10 -30 30])
z=-log(0.05)/sqrt(pi^2+(log(0.05)^2))
sgrid(z,0)

[k,poles]=rlocfind(C*G1)

%%
k = 1.0888e+08;
sys_cl=F*feedback(G1,k*C);

t=0:0.01:2;
step(0.1*sys_cl,t)
title('Closed-Loop Step Response w/ Notch Filter')

%% Frequency response controller design
m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

nump=[(m1+m2) b2 k2];
denp=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G1=tf(nump,denp);

num1=[-(m1*b2) -(m1*k2) 0 0];
den1=[(m1*m2) (m1*(b1+b2))+(m2*b1) (m1*(k1+k2))+(m2*k1)+(b1*b2) (b1*k2)+(b2*k1) k1*k2];
G2=tf(num1,den1);

numf=num1;
denf=nump;
F=tf(numf,denf);

%%
w = logspace(-1,2);
bode(G1,w)

%%
K=100000;
bode(K*G1,w) %not completed 


%% State feedback controller
m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

A=[0                 1   0                                              0
  -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)
   b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1
   k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0];
B=[0                 0
   1/m1              (b1*b2)/(m1*m2)
   0                -(b2/m2)
   (1/m1)+(1/m2)    -(k2/m2)];
C=[0 0 1 0];
D=[0 0];
sys=ss(A,B,C,D);

%% check controllability and observability

% checking observability
O = obsv(A , C)
Observability = rank(O); %if =4 then it is observable
if Observability == 4
    fprintf("System is observable\n")
else
    fprintf("System is unobservable")
end

% checking controllability 
CtrbMat = ctrb(A , B)
Controllability = rank(ctrb(A , B)); %if =4 then it is controllable

if Controllability == 4
    fprintf("System is controllable\n")
else
    fprintf("System is controllable")
end


%checking internal stability

e = eig(A) % if any of the EVs has positive real component, then the system is internally unstable




%checking external stability


% converting to TF
system = ss(A , B , C, D);
TF = tf(system);
TF
pole(TF) % if any of the poles has positive real component, then the system is externally (BIBO) unstable

%% Adding integral action to the SS model for zero steady state error

Aa=[0                 1   0                                              0         0
   -(b1*b2)/(m1*m2)   0   ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1)   -(b1/m1)   0
    b2/m2             0  -((b1/m1)+(b1/m2)+(b2/m2))                      1         0
    k2/m2             0  -((k1/m1)+(k1/m2)+(k2/m2))                      0         0
    0                 0   1                                              0         0];
Ba=[0                 0
    1/m1              (b1*b2)/(m1*m2)
    0                -(b2/m2)
    (1/m1)+(1/m2)    -(k2/m2)
    0                 0];
Ca=[0 0 1 0 0];
Da=[0 0];
sys=ss(Aa,Ba,Ca,Da);

%% controller gain original

K = [0 2.3e6 5e8 0 8e6]

%%
t = 0:0.01:2;
sys_cl = ss(Aa-Ba(:,1)*K,-0.1*Ba,Ca,Da);

p_original_withController = pole(sys_cl)

step(sys_cl*[0;1],t)
title('Closed-Loop Response to a 0.1-m Step')
hold on

%% controller gain design

p = pole(sys)
% p_new = [-23.9758, -23.9758 , -9 ,  -9 , -5]

p_new = [-28+0.02i, -28-0.02i , -12+0.01i ,  -12-0.01i , 0] * 100

Kmat = place(Aa,Ba,p_new)

Kmat(1,1) = 0
Kmat(1,4) = 0

%%
t = 0:0.01:2;
sys_cl = ss(Aa-Ba(:,1)*Kmat(1,:),-0.1*Ba,Ca,Da);


step(sys_cl*[0;1],t)
title('Closed-Loop Response to a 0.1-m Step')

hold off

%% 


