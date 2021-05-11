%%D:\Mega\Programming\
%clear

load Vehicle.mat VehicleA

tic
%totalTime = globalVar(0);% time in second
%period = globalVar(1);  %sampling period
%sensPeriod = globalVar(4);

fprintf("%s\nStarting FYP ...\n",datetime('now','Format','y-MMM-d HH-mm-ss'));

maxParallel = 10;    

for parallel = 1:maxParallel
    minReward = globalVar(5);
    fprintf("Starting Parallel...%d\n",parallel);
    maxWorker = 20;
    param = [0 0 0 0 0 0];
    prevResult = minReward;
    step = 0.001*2^10;
    prevWorker = maxWorker + 2;
    bestWorker = maxWorker + 2;
    flag = 0;
    %newParam =  zeros(maxWorker,6);
    %result = zeros(maxWorker,1);

    while(1)
        parfor worker = 1:maxWorker
            %fprintf("Running Worker %d\n",worker);
            if worker > 12 || prevResult == minReward
                newParam(worker,:) = (randi(16,1,6)-8);
                result(worker) = vehicleRunning(newParam(worker,:),VehicleA);
            else
                identity = eye(6);
                newParam(worker,:) = param + (-1)^(worker) * step *...
                               identity(idivide(worker+1,int8(2)),:);
                result(worker) = vehicleRunning(newParam(worker,:),VehicleA);
            end
        end
        bestWorkerOld = bestWorker;
        [maxResult,bestWorker] = max(result);
        if maxResult ~= minReward
            if maxResult > prevResult
                prevResult = maxResult;
                prevWorker = bestWorker;
                param = newParam(bestWorker,:);
                fprintf("%.2f,\t",param);
                fprintf("%.2f,\n",maxResult);
                flag = 1;
            elseif (idivide(bestWorker-1,int8(2)) == idivide(prevWorker-1,int8(2)) ||...
                (bestWorkerOld == bestWorker && bestWorker < maxWorker + 2)) ...
                && flag == 1
                step = step/2;
                if step <= 0.001
                    %fprintf("low step = %f\n",step);
                    break
                end
                fprintf("step = %f\n",step);
            else 
                fprintf("Worker %d,%d\n",bestWorker,prevWorker);
            end
        else
            
        end

    end
 
fprintf("%.2f,\t",param);
resultSave(param,prevResult);
%{
u = param(1);
v = param(2);
w = param(3);
x = param(4);
y = param(5);
z = param(6);


%%Plotting Result

clearvars VehicleB VBCont VehicleC VCCont 
clearvars VehicleD VDCont VehicleE VECont


VehicleB = Vehicle(1055,3,0,3,-6,5,23);
VehicleC = Vehicle(1055,3,0,3,-6,5,16);
VehicleD = Vehicle(1055,3,0,3,-6,5,9);
VehicleE = Vehicle(1055,3,0,3,-6,5,2);
VBCont = AccController(VehicleB,7.92,-0.352,2.96,8.72,0,0);
VCCont = AccController(VehicleC,u,v,w,x,y,z);
VDCont = AccController(VehicleD,u,v,w,x,y,z);
VECont = AccController(VehicleE,u,v,w,x,y,z);

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
date = datetime('now','Format','y-MMM-d HH-mm-ss');
imgLoc = sprintf("FYP/Image/%s/%.3f,%.3f,%.3f,%.3f,%.3f,%.3f/",...
    date,u,v,w,x,y,z);

close all

fig1 = figure(1);
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
imageName = append(imgLoc,"fig1.jpg");
saveas(fig1,imageName);
hold off


fig2 = figure(2);
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
iimageName = append(imgLoc,"fig2.jpg");
saveas(fig2,imageName);
hold off

fig3 = figure(3);
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
imageName = append(imgLoc,"fig3.jpg");
saveas(fig3,imageName);
hold off


fig4 = figure(4);
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
imageName = append(imgLoc,"fig4.jpg");
saveas(fig4,imageName);
hold off

fig5 = figure(5);
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
imageName = append(imgLoc,"fig5.jpg");
saveas(fig5,imageName);
hold off

%}
disp('done')
fprintf('time:%f\n',toc)

    for beepCount = 1:3
        beep
        pause(1)
    end
end

for beepCount = 1:20
        beep
        pause(1)
end


