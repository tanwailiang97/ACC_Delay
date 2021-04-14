%%D:\Mega\Programming\
clear
totalTime = 200;% time in second

VehicleA = Vehicle(1055,2.306,0.01,2.87,6);
for x = 1: (totalTime/globalVar(1)-1)
    VehicleA.move(5)
end

timePlot = 0:globalVar(1):(totalTime-20);

figure(1);
plot(timePlot,VehicleA.acc((1/globalVar(1):(totalTime-19)/globalVar(1))));
figure(2);
plot(VehicleA.vel);
figure(3);
plot(VehicleA.pos);

vel = zeros(1,20);
acc = vel;
for x = 3:20
    fprintf("%.3f ",gradient(gradient(VehicleA.pos(1:x))));
    disp('\n')
end