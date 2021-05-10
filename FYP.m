%%D:\Mega\Programming\
%clear
load Vehicle.mat VehicleA

tic
totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period
sensPeriod = globalVar(4);

parfor n = 1:4
    result(n,:) = bruteForce(n);
end

for n1 = 1:4
    for n2 = 1:length(result(n1,:))
        fprintf("%.4f,",result(n1,n2));
    end
    fprintf("\n\n");
end

[w,x,y,z,z1,z2] = result(1,1:6);
%%Plotting Result
clearvars VehicleB VBCont VehicleC VCCont 
clearvars VehicleD VDCont VehicleE VECont


VehicleB = Vehicle(1055,3,0,3,-6,5,23);
VehicleC = Vehicle(1055,3,0,3,-6,5,16);
VehicleD = Vehicle(1055,3,0,3,-6,5,9);
VehicleE = Vehicle(1055,3,0,3,-6,5,2);
VBCont = AccController(VehicleB,7.92,-0.352,2.96,8.72,0,0);
VCCont = AccController(VehicleC,w,x,y,z,z1,z2);
VDCont = AccController(VehicleD,w,x,y,z,z1,z2);
VECont = AccController(VehicleE,w,x,y,z,z1,z2);

vehBAcc = 0;
vehCAcc = 0;
vehDAcc = 0;
vehEAcc = 0;


for a = 1:(totalTime/period-1)

    VehicleB.move(vehBAcc);
    VehicleC.move(vehCAcc);
    VehicleD.move(vehDAcc);
    VehicleE.move(vehEAcc);

    if a > 1
        vehPDisC = VehicleB.pos(a-1) - VehicleC.pos(a-1) -5 ;
        vehPDisD = VehicleC.pos(a-1) - VehicleD.pos(a-1) -5 ;
        vehPDisE = VehicleD.pos(a-1) - VehicleE.pos(a-1) -5 ;
        
        vehPDisB = VehicleA.pos(a-1) - VehicleB.pos(a-1) -5 ;
        vehPVelB = VehicleA.vel(a-1) - VehicleB.vel(a-1);
        vehBAcc = VBCont.getAcc(0,vehPDisB,0,vehPVelB);
    end
    %%Sensor New Data
    if not(mod(a,sensPeriod/period))
        vehPVelC = VehicleB.vel(a-1) - VehicleC.vel(a-1);
        vehPVelD = VehicleC.vel(a-1) - VehicleD.vel(a-1);
        vehPVelE = VehicleD.vel(a-1) - VehicleE.vel(a-1);

        vehLDisC = VehicleA.pos(a-1) - VehicleC.pos(a-1) -5 ;
        vehLDisD = VehicleB.pos(a-1) - VehicleD.pos(a-1) -5 ;
        vehLDisE = VehicleC.pos(a-1) - VehicleE.pos(a-1) -5 ;

        vehLVelC = VehicleA.vel(a-1) - VehicleC.vel(a-1);
        vehLVelD = VehicleB.vel(a-1) - VehicleD.vel(a-1);
        vehLVelE = VehicleC.vel(a-1) - VehicleE.vel(a-1);

        vehCAcc = VCCont.getAcc(vehLDisC,vehPDisC,vehLVelC,vehPVelC); 
        vehDAcc = VDCont.getAcc(vehLDisD,vehPDisD,vehLVelD,vehPVelD);
        vehEAcc = VECont.getAcc(vehLDisE,vehPDisE,vehLVelE,vehPVelE);
    end
end

timePlot = 0:period:(totalTime-20);

close all

figure(1);
hold on
plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleD.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleE.acc((1/period:(totalTime-19)/period)));
title('accleration');
xlabel('time(s)');
ylabel('acceleration(ms-2)');
legend('A','B','C','D','E');
hold off


figure(2);
hold on
plot(timePlot,VehicleA.vel((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.vel((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.vel((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleD.vel((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleE.vel((1/period:(totalTime-19)/period)));
title('velocity');
xlabel('time(s)');
ylabel('velocity(ms-1)');
legend('A','B','C','D','E');
hold off

figure(3);
hold on
plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleD.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleE.pos((1/period:(totalTime-19)/period)));
title('position');
xlabel('time(s)');
ylabel('position(m)');
legend('A','B','C','D','E');
hold off


figure(4);
hold on
plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period))...
            -VehicleB.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.pos((1/period:(totalTime-19)/period))...
            -VehicleC.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.pos((1/period:(totalTime-19)/period))...
            -VehicleD.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleD.pos((1/period:(totalTime-19)/period))...
            -VehicleE.pos((1/period:(totalTime-19)/period)));
title('position different');
legend('B','C','D','E');
hold off

figure(5);
hold on
plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period))...
            -VehicleB.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.acc((1/period:(totalTime-19)/period))...
            -VehicleC.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.acc((1/period:(totalTime-19)/period))...
            -VehicleD.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleD.acc((1/period:(totalTime-19)/period))...
            -VehicleE.acc((1/period:(totalTime-19)/period)));
title('accleration different');
legend('B','C','D','E');
hold off

%{
figure(6)
hold on
plot(timePlot,VehicleB.jerk((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.jerk((1/period:(totalTime-19)/period)));
title('jerk');
legend('B','C');
hold off
%}
disp('done')
fprintf('time:%f\n',toc)
