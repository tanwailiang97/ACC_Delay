function [reward] = vehicleRunning(param,vehicleA)
    %fprintf("Running Vehicle\n");
    %load Vehicle.mat VehicleA
    VehicleA = vehicleA;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = globalVar(4);
    minReward = globalVar(5);
    u = param(1);
    v = param(2);
    w = param(3);
    x = param(4);
    y = param(5);
    z = param(6);
    VehicleB = Vehicle(1055,3,0,3,-6,5,23);
    VehicleC = Vehicle(1055,3,0,3,-6,5,16);
    VehicleD = Vehicle(1055,3,0,3,-6,5,9);
    VehicleE = Vehicle(1055,3,0,3,-6,5,2);
    VBCont = AccController(VehicleB,7.92,-0.352,2.96,8.72,0,0);
    VCCont = AccController(VehicleC,u,v,w,x,y,z);
    VDCont = AccController(VehicleD,u,v,w,x,y,z);
    VECont = AccController(VehicleE,u,v,w,x,y,z);
    
    %maxDisB = 0;
    maxDisC = 0;
    %maxDisD = 0;
    maxDisE = 0;
    vehBAcc = 0;
    vehCAcc = 0;
    vehDAcc = 0;
    vehEAcc = 0;
    %vehPDisB = 0;
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
                    50^(maxAcc / 2.5) * 50^(-minAcc / 4.5);
            if vehPDisC < 0 || vehPDisD < 0 || vehPDisE < 0
                reward = minReward;
                return
            end
        end
    end

end