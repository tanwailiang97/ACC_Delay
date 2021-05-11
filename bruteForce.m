function [w,x,y,z,z1,maxDisC,maxDisE,minAcc,maxAcc,dampCount,reward,tocTotal] = bruteForce(n)
    fprintf("Starting %d...",n);
    tic 
    load Vehicle.mat VehicleA
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = globalVar(4);

    %maxAcc = max(VehicleA.acc);
    %minAcc = min(VehicleA.acc);
    maxReward = -99999999;
    result = [0 0 0 0 0];

    wS = 0;
    xS = 0;
    yS = 0;
    zS = 0;
    z1S = 0;
    %z2S = 0;
    
    bound = [9 0.9 0.15];
    gap = [1.8 0.3 0.1];
    %minDamp = 999;
    
    for s = 1:3
        if s == 1
            if n == 1
                wTemp = (wS-bound(s)):gap(s):(wS-bound(s)/2);
            elseif n == 2
                wTemp = (wS-bound(s)/2):gap(s):0;
            elseif n == 3
                wTemp = 0:gap(s):(wS+bound(s)/2);
            elseif n == 4
                wTemp = (wS+bound(s)/2):gap(s):(wS+bound(s));
            end       
        else
            wTemp = (wS-bound(s)):gap(s):(wS+bound(s));
        end
        for w = wTemp
            for x = (xS-bound(s)):gap(s):(xS+bound(s))
                for y = (yS-bound(s)):gap(s):(yS+bound(s))
                    for z = (zS-bound(s)):gap(s):(zS+bound(s))
                        for z1 = (z1S-bound(s)):gap(s):(z1S+bound(s))
                            %for z2 = (z2S-bound(s)):gap(s):(z2S+bound(s))
                                clearvars VehicleB VBCont VehicleC VCCont 
                                clearvars VehicleD VDCont VehicleE VECont

                                VehicleB = Vehicle(1055,3,0,3,-6,5,23);
                                VehicleC = Vehicle(1055,3,0,3,-6,5,16);
                                VehicleD = Vehicle(1055,3,0,3,-6,5,9);
                                VehicleE = Vehicle(1055,3,0,3,-6,5,2);
                                VBCont = AccController(VehicleB,7.92,-0.352,2.96,8.72,0,0);
                                VCCont = AccController(VehicleC,0,w,x,y,z,z1);
                                VDCont = AccController(VehicleD,0,w,x,y,z,z1);
                                VECont = AccController(VehicleE,0,w,x,y,z,z1);
                                flag = 9;
                                %maxDisB = 0;
                                maxDisC = 0;
                                %maxDisD = 0;
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
                                    VehicleD.move(vehDAcc);
                                    VehicleE.move(vehEAcc);

                                    if a > 1
                                        vehPDisB = VehicleA.pos(a-1) - VehicleB.pos(a-1) -5 ;
                                        vehPDisC = VehicleB.pos(a-1) - VehicleC.pos(a-1) -5 ;
                                        vehPDisD = VehicleC.pos(a-1) - VehicleD.pos(a-1) -5 ;
                                        vehPDisE = VehicleD.pos(a-1) - VehicleE.pos(a-1) -5 ;
                                        
                                        

                                        vehPVelB = VehicleA.vel(a-1) - VehicleB.vel(a-1);
                                        vehBAcc = VBCont.getAcc(0,vehPDisB,0,vehPVelB);
                                    end
                                    %%Sensor New Data
                                    if not(mod(a,sensPeriod/period))
                                        vehPVelC = VehicleB.vel(a-2) - VehicleC.vel(a-2);
                                        vehPVelD = VehicleC.vel(a-2) - VehicleD.vel(a-2);
                                        vehPVelE = VehicleD.vel(a-2) - VehicleE.vel(a-2);
                                        
                                        vehLDisC = VehicleA.pos(a-1) - VehicleC.pos(a-1) -5 ;
                                        vehLDisD = VehicleB.pos(a-1) - VehicleD.pos(a-1) -5 ;
                                        vehLDisE = VehicleC.pos(a-1) - VehicleE.pos(a-1) -5 ;
                                        
                                        vehLVelC = VehicleA.vel(a-2) - VehicleC.vel(a-2);
                                        vehLVelD = VehicleB.vel(a-2) - VehicleD.vel(a-2);
                                        vehLVelE = VehicleC.vel(a-2) - VehicleE.vel(a-2);
                                        
                                        vehCAcc = VCCont.getAcc(vehLDisC,vehPDisC,vehLVelC,vehPVelC); 
                                        vehDAcc = VDCont.getAcc(vehLDisD,vehPDisD,vehLVelD,vehPVelD);
                                        vehEAcc = VECont.getAcc(vehLDisE,vehPDisE,vehLVelE,vehPVelE);
                                    end
                                    if vehPDisC > maxDisC
                                        maxDisC = vehPDisC;
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
                                        reward =  -(maxDisC + maxDisE) * dampCount * ...
                                                20^(maxAcc / 2.5) * 20^(-minAcc / 4.5);
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
                                    %minDamp = dampCount;
                                    result = [w x y z z1];
                                    fprintf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t> %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n"...
                                        ,w,x,y,z,z1,z2,maxDisC,maxDisE,minAcc,maxAcc,dampCount,reward);
                                else
                                    fprintf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t> Err %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n"...
                                        ,w,x,y,z,z1,z2,maxDisC,maxDisE,minAcc,maxAcc,dampCount,reward);
                                end
                            %end
                        end
                    end
                end
            end
        end
    
    wS = result(1);
    xS = result(2);
    yS = result(3);
    zS = result(4);
    z1S = result(5);
    %z2S = result(6);
    disp(result);
    fprintf('time w:%f\n',toc)
    end
    
    tocTotal = toc;
    fprintf('time w:%f\n',toc)
end

