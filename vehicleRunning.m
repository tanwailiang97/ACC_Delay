function [reward] = vehicleRunning(param,vehicleA,vehicleB,accF,disF,sensD)

    VehicleA = vehicleA;
    VehicleB = vehicleB;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = sensD;
    sensSamp = round(sensPeriod/period);
    minReward = globalVar(5);
    delL = round(sensD/period);
    delP = round(sensD/period);
    accFactor = accF;
    disFactor = disF;
    tao = globalVar(14);
    dampFactor = globalVar(10);
    delComp = globalVar(8);
    extraDis = globalVar(9);
    st = globalVar(11);
    sens2 = globalVar(13);
    
    u = param(1);
    v = param(2);
    w = param(3);
    x = param(4);
    y = param(5);
    z = param(6);
    
    if w < 0 || x < 0 || y < 0 || z < 0 || v > 0
        reward = minReward;
        return
    end

    VehicleC = Vehicle(1055,3,tao,2,-2,0,20);
    VehicleD = Vehicle(1055,3,tao,2,-2,0,15);

    VCCont = AccController(VehicleC,u,v,w,x,y,z);
    VDCont = AccController(VehicleD,u,v,w,x,y,z);
    
    VCDeComp = DelayComp(VehicleC,1,1,sensD);
    VDDeComp = DelayComp(VehicleD,1,1,sensD);

    vehCAcc = 0;
    vehDAcc = 0;

    dampCount = 1;
    dampCount2 = 1;
    
    
    for a = 1:(totalTime/period)

        VehicleC.move(vehCAcc,a);
        VehicleD.move(vehDAcc,a);
        
        if (a > delP) && (a > delL)
            
            vehPDisC = VehicleB.pos(a-delP) - VehicleC.pos(a-delP) -3 ;
            vehPDisD = VehicleC.pos(a-delP) - VehicleD.pos(a-delP) -3 ;
            %%Crash
            if vehPDisC <= 0 || vehPDisD <= 0
                reward = minReward;
                %fprintf("%d-Crashed\n",a);
                return
            end
            %%Sensor New Data
            if  (not(mod(a,sensSamp)) || (sens2 && not(mod((a-(sensSamp/2)^(1-delComp)),sensSamp))) )
                if sens2
                    if not(mod(a,sensSamp))
                        state = 0;
                    else
                        state = 1;
                    end
                else
                    state = 2;
                end
                
                vehPVelC = VehicleB.vel(a-delP) - VehicleC.vel(a-delP);
                vehPVelD = VehicleC.vel(a-delP) - VehicleD.vel(a-delP);

                vehLDisC = VehicleA.pos(a-delL) - VehicleC.pos(a-delL) -6 ;
                vehLDisD = VehicleB.pos(a-delL) - VehicleD.pos(a-delL) -6 ;

                vehLVelC = VehicleA.vel(a-delL) - VehicleC.vel(a-delL);
                vehLVelD = VehicleB.vel(a-delL) - VehicleD.vel(a-delL);
                
                if delComp
                    [vehPVelC,vehPDisC] = VCDeComp.get(vehPVelC,vehPDisC,a,state,VehicleB.acc(a-delL));
                    [vehPVelD,vehPDisD] = VDDeComp.get(vehPVelD,vehPDisD,a,state,VehicleC.acc(a-delL));
                    
                    [vehLVelC,vehLDisC] = VCDeComp.get(vehLVelC,vehLDisC,a,state,VehicleA.acc(a-delL));
                    [vehLVelD,vehLDisD] = VDDeComp.get(vehLVelD,vehLDisD,a,state,VehicleB.acc(a-delL));
                    
                end
                
                if (~delComp) || state
                    vehCAcc = VCCont.getAcc(vehLDisC-2*extraDis,vehPDisC-extraDis,vehLVelC,vehPVelC,a); 
                    vehDAcc = VDCont.getAcc(vehLDisD-2*extraDis,vehPDisD-extraDis,vehLVelD,vehPVelD,a);
                end
                
            end
        end
        
        %%Condition check
        if a > st/period
            if (VehicleC.acc(end)-VehicleC.acc(end-1))*((-1)^dampCount) > 0
                dampCount = dampCount + 1;
            end
            if (VehicleD.acc(end)-VehicleD.acc(end-1))*((-1)^dampCount2) > 0
                dampCount2 = dampCount2 + 1;
            end
        end
        
    end
    
    maxAcc = max(VehicleD.acc);
    minAcc = min(VehicleD.acc);

    maxDisC = max(VehicleB.pos-VehicleC.pos);
    maxDisD = max(VehicleC.pos-VehicleD.pos);
    
    accIncrease =  1000*(0.001 + max(VehicleD.acc) -  max(VehicleB.acc))* ...
                   1000*(0.001 - min(VehicleD.acc) +  min(VehicleB.acc));
               
    if accIncrease < 0.9999
        accIncrease = 0.9999;
    end
    if maxAcc < 1.109
        maxAcc = 1.109;
    elseif maxAcc > 1.1202
        maxAcc = 30;
    end
    if minAcc > -0.549
        minAcc = -0.549;
    end
    
                
    
    reward =  -((maxDisC * maxDisD )^ disFactor) * ...
                        (dampCount + dampCount2)^dampFactor * ...
                        accFactor^( 1 + abs(maxAcc) + abs(minAcc)) * ...
                        (accIncrease) ^ 5;
    
end