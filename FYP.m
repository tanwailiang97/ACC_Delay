%%D:\Mega\Programming\
clear
%load Vehicle1.mat VehicleA
totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period


timePlot = 0:period:(totalTime-20);

figure(1);
plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period)));
figure(2);
plot(timePlot,VehicleA.vel((1/period:(totalTime-19)/period)));
figure(3);
plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period)));
