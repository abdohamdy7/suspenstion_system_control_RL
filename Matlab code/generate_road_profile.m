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
plot(x,hx)

rough_road_simin.time=x';
rough_road_simin.signals.values=hx';

