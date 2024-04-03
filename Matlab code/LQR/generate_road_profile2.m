% generate road profile two bumps


c=0;
a1=0.2;
a2= 0.3;
sim=60;
Ts=0.01;
for i = 0:Ts:sim
    c=c+1;
    if i>=16 && i<=32
        ud(c)=a1*(1-cos(0.125*pi*i));
    % elseif i>5 && i<6
        %ud(c)=a2*(1-cos(2*pi*i));
    else
        ud(c)=0;

    end
end

i = 0:Ts:sim;
plot(i,ud)


pump_road_simin.time=i';
pump_road_simin.signals.values=ud';

