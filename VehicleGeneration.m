totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period

VehicleA = Vehicle(1055,2.306,0.01,3,-6);
VehicleA.pos(1) = 30;


for x = 1: (totalTime/globalVar(1)-1)
    if(x < 1/globalVar(1))
        VehicleA.move(0);
    elseif (x < 12/globalVar(1))  %10 sec acclerate
        VehicleA.move(2.5);
    elseif (x < 22/globalVar(1))  %10 sec stable
        VehicleA.move(0);
    elseif (x < 26/globalVar(1))  %10 sec acclerate
        VehicleA.move(-5);
    elseif (x < 34/globalVar(1))  %10 sec acclerate
        VehicleA.move(2.5);
    elseif (x < 38/globalVar(1))  %10 sec acclerate
        VehicleA.move(-5);
    else
        VehicleA.move(0);
    end
end

timePlot = 0:globalVar(1):(totalTime-20);

figure(1);
plot(timePlot,VehicleA.acc((1/globalVar(1):(totalTime-19)/globalVar(1))));
figure(2);
plot(VehicleA.vel);
figure(3);
plot(VehicleA.pos);
