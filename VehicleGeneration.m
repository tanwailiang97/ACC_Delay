function [VehicleA,VehicleB] = VehicleGeneration(velInc,velDec)

totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period

VehicleA = Vehicle(1055,2.306,0.01,1.962,-1.962,0,30);
VehicleB = Vehicle(1055,2.306,0.01,1.962,-1.962,0,25);

%{
for x = 1: (totalTime/period)
    k = -1/4.5; 
    y = x * period;
    if y <= 40
        VehicleA.acc(x) = velInc*(-k*exp(k*y));
    elseif y <= 80
        VehicleA.acc(x) = velInc*(-k*exp(k*y)*exp(-40*k));
    elseif y <= 120 
        VehicleA.acc(x) = velDec*(-k*exp(k*y)*exp(-80*k));
    elseif y <= 160
        VehicleA.acc(x) = velInc*(-k*exp(k*y)*exp(-120*k));
    end
end


VehicleA.jerk = gradient(VehicleA.acc)/VehicleA.period;
VehicleA.vel  = cumtrapz(VehicleA.acc)*period;
VehicleA.pos  = cumtrapz(VehicleA.vel)*period + VehicleA.offPos;
%}

for x = 1: (totalTime/period)
    k = -1/6.86;    %38s
    y = x * period - 10;
    if y <= 0
        VehicleA.vel(x) = 0;
    elseif y <= 2
        VehicleA.vel(x) = velInc*0.1272*((y)/2)^2;
    elseif y <= 40
        VehicleA.vel(x) = VehicleA.vel(12/period) +  velInc*0.8728*(1-exp(k*(y-2)));
    elseif y <= 42
        VehicleA.vel(x) = VehicleA.vel(50/period) + velInc*0.1272*((y-40)/2)^2;
    elseif y <= 80
        VehicleA.vel(x) = VehicleA.vel(52/period) + velInc*0.8728*(1-exp(k*(y-42)));
    elseif y <= 82
        VehicleA.vel(x) = VehicleA.vel(90/period) + velDec*0.1272*((y-80)/2)^2;
    elseif y <= 120 
        VehicleA.vel(x) = VehicleA.vel(92/period) + velDec*0.8728*(1-exp(k*y)*exp(-82*k));
    elseif y <= 122
        VehicleA.vel(x) = VehicleA.vel(130/period) + velInc*0.1272*((y-120)/2)^2;
    elseif y <= 160 
        VehicleA.vel(x) = VehicleA.vel(132/period) + velInc*0.8728*(1-exp(k*y)*exp(-122*k));
    end
end

VehicleA.jerk = del2(VehicleA.vel,period);
VehicleA.acc  = gradient(VehicleA.vel)/period;
VehicleA.pos  = cumtrapz(VehicleA.vel)*period + VehicleA.offPos;



VehicleB.pos = VehicleA.pos - 5 - VehicleA.vel * 0.4;
VehicleB.vel = gradient(VehicleB.pos)/period;
VehicleB.acc = gradient(VehicleB.vel)/period;

%VehicleB.vel = VehicleA.vel;
%VehicleB.pos = cumtrapz(VehicleB.vel)*period + VehicleB.offPos;
%VehicleB.acc = gradient(VehicleB.vel)/VehicleB.period;
