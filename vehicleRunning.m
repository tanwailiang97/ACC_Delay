function [reward] = vehicleRunning(param,vehicleA,accF,disF)
    %fprintf("Running Vehicle\n");
    %load Vehicle.mat VehicleA
    VehicleA = vehicleA;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = globalVar(4);
    minReward = globalVar(5);
    delL = ceil(globalVar(2)/period);
    delP = ceil(globalVar(3)/period);
    accFactor = accF;
    disFactor = disF;
    
    u = param(1);
    v = param(2);
    w = param(3);
    x = param(4);
    y = param(5);
    z = param(6);
    
    VehicleB = Vehicle(1055,3,0,3,-6,5,23);
    VehicleC = Vehicle(1055,3,0,3,-6,5,13);
    VehicleD = Vehicle(1055,3,0,3,-6,5,3);
    VehicleE = Vehicle(1055,3,0,3,-6,5,-7);
    VehicleF = Vehicle(1055,3,0,3,-6,5,-17);
    VehicleG = Vehicle(1055,3,0,3,-6,5,-27);
    
    VBCont = AccController(VehicleB,7.92,-0.352,2.96,8.72,0,0);
    VCCont = AccController(VehicleC,u,v,w,x,y,z);
    VDCont = AccController(VehicleD,u,v,w,x,y,z);
    VECont = AccController(VehicleE,u,v,w,x,y,z);
    VFCont = AccController(VehicleF,u,v,w,x,y,z);
    VGCont = AccController(VehicleG,u,v,w,x,y,z);
    
    
    maxDisC = 0;
    maxDisD = 0;
    maxDisE = 0;
    maxDisF = 0;
    maxDisG = 0;
    
    vehBAcc = 0;
    vehCAcc = 0;
    vehDAcc = 0;
    vehEAcc = 0;
    vehFAcc = 0;
    vehGAcc = 0;
    
    vehPDisC = 0;
    vehPDisD = 0;
    vehPDisE = 0;
    vehPDisF = 0;
    vehPDisG = 0;
    
    maxAcc = 2;
    minAcc = -4.5;
    dampCount = 1;
    
    
    for a = 1:(totalTime/period-1)

        VehicleB.move(vehBAcc);
        VehicleC.move(vehCAcc);
        VehicleD.move(vehDAcc);
        VehicleE.move(vehEAcc);
        VehicleF.move(vehFAcc);
        VehicleG.move(vehGAcc);
        if a > 1
            vehPDisB = VehicleA.pos(a-1) - VehicleB.pos(a-1) -5 ;
            vehPVelB = VehicleA.vel(a-1) - VehicleB.vel(a-1);
            vehBAcc = VBCont.getAcc(0,vehPDisB,0,vehPVelB); 
        end
        if (a > delP) && (a > delL)
            
            vehPDisC = VehicleB.pos(a-delP) - VehicleC.pos(a-delP) -5 ;
            vehPDisD = VehicleC.pos(a-delP) - VehicleD.pos(a-delP) -5 ;
            vehPDisE = VehicleD.pos(a-delP) - VehicleE.pos(a-delP) -5 ;
            vehPDisF = VehicleE.pos(a-delP) - VehicleF.pos(a-delP) -5 ;
            vehPDisG = VehicleF.pos(a-delP) - VehicleG.pos(a-delP) -5 ;
            
            %%Sensor New Data
            if not(mod(a,sensPeriod/period))
                vehPVelC = VehicleB.vel(a-delP) - VehicleC.vel(a-delP);
                vehPVelD = VehicleC.vel(a-delP) - VehicleD.vel(a-delP);
                vehPVelE = VehicleD.vel(a-delP) - VehicleE.vel(a-delP);
                vehPVelF = VehicleE.vel(a-delP) - VehicleF.vel(a-delP);
                vehPVelG = VehicleF.vel(a-delP) - VehicleG.vel(a-delP);

                vehLDisC = VehicleA.pos(a-delL) - VehicleC.pos(a-delL) -5 ;
                vehLDisD = VehicleB.pos(a-delL) - VehicleD.pos(a-delL) -5 ;
                vehLDisE = VehicleC.pos(a-delL) - VehicleE.pos(a-delL) -5 ;
                vehLDisF = VehicleD.pos(a-delL) - VehicleF.pos(a-delL) -5 ;
                vehLDisG = VehicleE.pos(a-delL) - VehicleG.pos(a-delL) -5 ;

                vehLVelC = VehicleA.vel(a-delL) - VehicleC.vel(a-delL);
                vehLVelD = VehicleB.vel(a-delL) - VehicleD.vel(a-delL);
                vehLVelE = VehicleC.vel(a-delL) - VehicleE.vel(a-delL);
                vehLVelF = VehicleD.vel(a-delL) - VehicleF.vel(a-delL);
                vehLVelG = VehicleE.vel(a-delL) - VehicleG.vel(a-delL);

                vehCAcc = VCCont.getAcc(vehLDisC,vehPDisC,vehLVelC,vehPVelC); 
                vehDAcc = VDCont.getAcc(vehLDisD,vehPDisD,vehLVelD,vehPVelD);
                vehEAcc = VECont.getAcc(vehLDisE,vehPDisE,vehLVelE,vehPVelE);
                vehFAcc = VFCont.getAcc(vehLDisF,vehPDisF,vehLVelF,vehPVelF);
                vehGAcc = VGCont.getAcc(vehLDisG,vehPDisG,vehLVelG,vehPVelG);
            end
        end
        
        if vehPDisC > maxDisC
            maxDisC = vehPDisC;
        end
        if vehPDisD > maxDisD
            maxDisD = vehPDisD;
        end
        if vehPDisE > maxDisE
            maxDisE = vehPDisE;
        end
        if vehPDisF > maxDisF
            maxDisF = vehPDisF;
        end
        if vehPDisG > maxDisG
            maxDisG = vehPDisG;
        end
        %%Condition check
        if a > 1/period

            if VehicleG.acc(a-2) > maxAcc 
                maxAcc = VehicleG.acc(a-2);
            elseif VehicleG.acc(a-2) < minAcc
                minAcc = VehicleG.acc(a-2);
            end
            
            %if (VehicleE.acc(a-2)-VehicleE.acc(a-3))*((-1)^dampCount) > 0
            %    dampCount = dampCount + 1;
            %end
            
            reward =  -((maxDisC + maxDisD + maxDisE + maxDisF + maxDisG)^ disFactor) * ...
                        dampCount * ...
                        accFactor^(maxAcc / 2.5 - minAcc / 4.5);
            if vehPDisC < 0 || vehPDisD < 0 || vehPDisE < 0
                reward = minReward;
                return
            end
        end
    end

end