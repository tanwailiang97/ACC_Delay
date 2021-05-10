%%D:\Mega\Programming\
%clear
load Vehicle.mat VehicleA

tic
totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period
sensPeriod = globalVar(4);



parfor n = 1:4
    result(n) = bruteForce(n);
end

for n1 = 1:2
    for n2 = 1:length(result(n1),:)
        fprintf("%.4f",result(n1,n2));
    end
    fprintf("\n\n");
end
fprintf("%.2f\t%.2f\t%.2f\t%.2f\t> %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n"...
                            ,w,x,y,z,maxDisD,maxDisE,minAcc,maxAcc,dampCount,reward);
%{
%maxAcc = max(VehicleA.acc);
%minAcc = min(VehicleA.acc);
maxReward = -9999999999;
result = [0 0 0 0];

wS = 0;
xS = 0;
yS = 0;
zS = 0;
bound = [8.1 0.9 0.15];
gap = [1.8 0.3 0.1];
minDamp = 999;

for s = 1:3
    for w = (wS-bound(s)):gap(s):(wS+bound(s))
        for x = (xS-bound(s)):gap(s):(xS+bound(s))
            for y = (yS-bound(s)):gap(s):(yS+bound(s))
                for z = (zS-bound(s)):gap(s):(zS+bound(s))
                    clearvars VehicleB VBCont VehicleC VCCont

                    VehicleB = Vehicle(1055,3,0,3,-6,5,23);
                    VehicleC = Vehicle(1055,3,0,3,-6,5,16);
                    VehicleD = Vehicle(1055,3,0,3,-6,5,9);
                    VehicleE = Vehicle(1055,3,0,3,-6,5,2);
                    VBCont = AccController(VehicleB,w,x,y,z,0,0);
                    VCCont = AccController(VehicleC,w,x,y,z,0,0);
                    VDCont = AccController(VehicleC,w,x,y,z,0,0);
                    VECont = AccController(VehicleC,w,x,y,z,0,0);
                    flag = 9;
                    maxDisB = 0;
                    maxDisC = 0;
                    maxDisD = 0;
                    maxDisE = 0;
                    vehBAcc = 0;
                    vehCAcc = 0;
                    vehDAcc = 0;
                    vehEAcc = 0;
                    vehPDisB = 0;
                    vehPDisC = 0;
                    vehPDisD = 0;
                    vehPDisE = 0;
                    maxAcc = 2;
                    minAcc = -4.5;
                    dampCount = 1;

                    for a = 1:(totalTime/period-1)

                        VehicleB.move(vehBAcc);
                        VehicleC.move(vehCAcc);
                        VehicleD.move(vehCAcc);
                        VehicleE.move(vehCAcc);

                        if a > 1
                            vehPDisB = VehicleA.pos(a-1) - VehicleB.pos(a-1) -5 ;
                            vehPDisC = VehicleB.pos(a-1) - VehicleC.pos(a-1) -5 ;
                            vehPDisD = VehicleC.pos(a-1) - VehicleD.pos(a-1) -5 ;
                            vehPDisE = VehicleD.pos(a-1) - VehicleE.pos(a-1) -5 ;
                        end
                        %%Sensor New Data
                        if not(mod(a,sensPeriod/period))
                            vehPVelB = VehicleA.vel(a-2) - VehicleB.vel(a-2);
                            vehPVelC = VehicleB.vel(a-2) - VehicleC.vel(a-2);
                            vehPVelD = VehicleC.vel(a-2) - VehicleD.vel(a-2);
                            vehPVelE = VehicleD.vel(a-2) - VehicleE.vel(a-2);
                            vehBAcc = VBCont.getAcc(0,vehPDisB,0,vehPVelB);
                            vehCAcc = VCCont.getAcc(0,vehPDisC,0,vehPVelC); 
                            vehDAcc = VDCont.getAcc(0,vehPDisD,0,vehPVelD);
                            vehEAcc = VECont.getAcc(0,vehPDisE,0,vehPVelE);
                        end
                        %if vehPDisB > maxDisB
                        %    maxDisB = vehPDisB;
                        %end
                        %if vehPDisC > maxDisC
                        %    maxDisC = vehPDisC;
                        %end
                        if vehPDisD > maxDisD
                            maxDisD = vehPDisD;
                        end
                        if vehPDisE > maxDisE
                            maxDisE = vehPDisE;
                        end
                        %%Condition check
                        if a > 1/period
                            
                            if VehicleE.acc(a-2) > maxAcc 
                                maxAcc = VehicleE.acc(a-2);
                            elseif VehicleE.acc(a-2) < minAcc
                                minAcc = VehicleE.acc(a-2);
                            end
                            
                            if (VehicleE.acc(a-2)-VehicleE.acc(a-3))*((-1)^dampCount) > 0
                                dampCount = dampCount + 1;
                            end
                            reward =  -0.5*(maxDisD + maxDisE) * dampCount * ...
                                    20^(maxAcc / 2.5) * 10^(-minAcc / 4.5);
                            if vehPDisB < 0 || vehPDisC < 0 || vehPDisD < 0 || vehPDisE < 0
                                flag = 1;
                                continue
                            elseif reward < maxReward
                                flag = 2;
                                continue
                            %elseif dampCount > minDamp
                            %    flag = 3;
                            %    continue
                            elseif maxAcc > 2 && minAcc < 4
                                flag =0;
                            end
                            %elseif VehicleC.acc(a-2) > maxAcc || VehicleC.acc(a-2)< minAcc
                            %    flag = 3;
                            %    continue
                            
                        end
                    end

                    %%Reward Check
                    if flag == 1
                        %fprintf("%.2f\t%.2f\t%.2f\t%.2f\t= Co\n",w,x,y,z);%Collided
                    elseif flag == 2
                        %fprintf("%.2f\t%.2f\t%.2f\t%.2f\t= LR\n",w,x,y,z);%Low Reward
                    elseif flag == 3   
                        %fprintf("%.2f\t%.2f\t%.2f\t%.2f\t= Os\n",w,x,y,z);%UnderShoot
                    elseif(reward > maxReward) && flag == 0
                        maxReward = reward;
                        minDamp = dampCount;
                        result = [w x y z];
                        fprintf("%.2f\t%.2f\t%.2f\t%.2f\t> %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n"...
                            ,w,x,y,z,maxDisD,maxDisE,minAcc,maxAcc,dampCount,reward);
                    else
                        fprintf("%.2f\t%.2f\t%.2f\t%.2f\t= Err\n",w,x,y,z);
                    end
                end
                %fprintf('time z:%f\n',toc)
            end
            fprintf('time y:%f\n',toc)
        end
        fprintf('time x:%f\n',toc)
    end
    wS = result(1);
    xS = result(2);
    yS = result(3);
    zS = result(4);
    disp(result);
    fprintf('time w:%f\n',toc)
end
%}



%%Plotting Result
clearvars VehicleB VBCont VehicleC VCCont


VehicleB = Vehicle(1055,3,0,3,-6,5,23);
VehicleC = Vehicle(1055,3,0,3,-6,5,16);
VBCont = AccController(VehicleB,result(1),result(2),result(3),result(4),0,0);
VCCont = AccController(VehicleC,result(1),result(2),result(3),result(4),0,0);
for a = 1:(totalTime/period-1)

    VehicleB.move(vehBAcc);
    VehicleC.move(vehCAcc);

    if a > 2
        vehPDisB = VehicleA.pos(a-2) - VehicleB.pos(a-2) -5 ;
        vehPDisC = VehicleB.pos(a-2) - VehicleC.pos(a-2) -5 ;
    end
    %%Sensor New Data
    if not(mod(a,sensPeriod/period))
        vehPVelB = VehicleA.vel(a-2) - VehicleB.vel(a-2);
        vehPVelC = VehicleB.vel(a-2) - VehicleC.vel(a-2);
        vehBAcc = VBCont.getAcc(0,vehPDisB,0,vehPVelB);
        vehCAcc = VCCont.getAcc(0,vehPDisC,0,vehPVelC); 
    end

end

timePlot = 0:period:(totalTime-20);

close all

figure(1);
hold on
plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.acc((1/period:(totalTime-19)/period)));
title('accleration');
xlabel('time(s)');
ylabel('acceleration(ms-2)');
legend('A','B','C');
hold off


figure(2);
hold on
plot(timePlot,VehicleA.vel((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.vel((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.vel((1/period:(totalTime-19)/period)));
title('velocity');
xlabel('time(s)');
ylabel('velocity(ms-1)');
legend('A','B','C');
hold off

figure(3);
hold on
plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleC.pos((1/period:(totalTime-19)/period)));
title('position');
xlabel('time(s)');
ylabel('position(m)');
legend('A','B','C');
hold off


figure(4);
hold on
plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period))...
            -VehicleB.pos((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.pos((1/period:(totalTime-19)/period))...
            -VehicleC.pos((1/period:(totalTime-19)/period)));
title('position different');
legend('B','C');
hold off

figure(5);
hold on
plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period))...
            -VehicleB.acc((1/period:(totalTime-19)/period)));
plot(timePlot,VehicleB.acc((1/period:(totalTime-19)/period))...
            -VehicleC.acc((1/period:(totalTime-19)/period)));
title('accleration different');
legend('B','C');
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
