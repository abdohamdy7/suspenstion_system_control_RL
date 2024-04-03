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
O = obsv(A , C);
Observability = rank(O); %if =4 then it is observable
if Observability == 4
    fprintf("System is observable\n")
else
    fprintf("System is unobservable")
end

% checking controllability 

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

K = [0 2.3e6 5e8 0 8e6];

%%
t = 0:0.01:2;
sys_cl = ss(Aa-Ba(:,1)*K,-0.1*Ba,Ca,Da);
% 
% p_original = pole(sys_cl)
% 
% step(sys_cl*[0;1],t)
% title('Closed-Loop Response to a 0.1-m Step')
% hold on

%% controller gain design

p = pole(sys);
% p_new = [-23.9758, -23.9758 , -9 ,  -9 , -5]

p_new = [-28+0.02i, -28-0.02i , -12+0.01i ,  -12-0.01i , 0] * 100

Kmat = place(Aa,Ba,p_new);

Kmat(1,1) = 0;
Kmat(1,4) = 0;
Kmat
%%
t = 0:0.01:5;
s_cl = ss(Aa-Ba(:,1)*Kmat(1,:),-0.1*Ba,Ca,Da);


step(s_cl*[0;1],t)
title('Closed-Loop Response to a 0.1-m Step')

% hold off

%% 
t = 0:1/1e3:10;
d = [0:2:10]';
x = @rectpuls;
v = pulstran(t,d,x);

% plot(t,y)
% hold off
% xlabel('Time (s)')
% ylabel('Waveform')

%%

v = [zeros(1,numel(v)); v];
lsim(TF,v,t); %do the simulation
plot(t,y,t,v)
legend('Response','Input')

%%

k = 3; % Values For ISO Road C-D Roughness Classification, from 3 to 9
V=40; % Car Velocity Km/h
L  = 250;  % Length Of Road Profile (m)
N=L/(V/3.6)*100; %  Number of data points
B  = L/N ; % Sampling Interval (m)
dn = 1/L;  % Frequency Band
n0 = 0.1;        % Spatial Frequency (cycles/m)
n  = dn : dn : N*dn; % Spatial Frequency Band
phi =  2*pi*rand(size(n)); % Random Phase Angle
Amp1 = sqrt(dn)*(2^k)*(1e-3)*(n0./n); % Amplitude for Road  Class A-B
x = 0:B:L-B; % Abscissa Variable from 0 to L
hx = zeros(size(x));
for i=1:length(x)
    hx(i) = sum(Amp1.*cos(2*pi*n*x(i)+ phi));
end
% plot(x,hx)

varsimin.time=x';
varsimin.signals.values=hx';


