function [VehicleA,VehicleB] = VehicleGeneration(velInc,velDec)

totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period

VehicleA = Vehicle(1055,2.306,0.01,1.962,-1.962,0,30);
VehicleB = Vehicle(1055,2.306,0.01,1.962,-1.962,0,25);


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

%{
for x = 1: (totalTime/period)
    k = -1/6.86;
    y = x * period;
    if(y <= 40)
        VehicleA.vel(x) = 10*(1-exp(k*y));
    elseif (y <= 80) 
        VehicleA.vel(x) = VehicleA.vel(40/period) + 10*(1-exp(k*y)*exp(-40*k));
    elseif (y <= 120) 
        VehicleA.vel(x) = VehicleA.vel(80/period) - 5*(1-exp(k*y)*exp(-80*k));
    elseif (y <= 160) 
        VehicleA.vel(x) = VehicleA.vel(120/period) + 10*(1-exp(k*y)*exp(-120*k));
    end
end
%}

VehicleA.jerk = gradient(VehicleA.acc)/VehicleA.period;
VehicleA.vel  = cumtrapz(VehicleA.acc)*period;
VehicleA.pos  = cumtrapz(VehicleA.vel)*period + VehicleA.offPos;

VehicleB.pos = VehicleA.pos - VehicleA.vel*0.1 - 5;
VehicleB.vel = gradient(VehicleB.pos)/VehicleB.period;
VehicleB.acc = del2(VehicleB.pos,VehicleB.period);

%{
timePlot = globalVar(1):globalVar(1):(totalTime);

close all

figure;
hold on
plot(timePlot,VehicleA.jerk);
title("Jerk");
%plot(timePlot,VehicleB.acc);
%legend({'A','B'},'Location','southeast');
hold off

figure;
hold on
plot(timePlot,VehicleA.acc);
title("Acc");
%plot(timePlot,VehicleB.acc);
%legend({'A','B'},'Location','southeast');
hold off

figure;
hold on
plot(timePlot,VehicleA.vel);
%plot(timePlot,VehicleB.vel);
title("Vel");
%legend({'A','B'},'Location','southeast');
hold off

figure;
hold on
plot(timePlot,VehicleA.pos-VehicleB.pos);
title("Pos");
%plot(timePlot,VehicleB.pos);
%legend({'A','B'},'Location','southeast');
hold off
%}
%}