function [reward] = vehicleRunning(param,vehicleA,accF,disF)
    
    %count = 1;
    
    %fprintf("Running Vehicle\n");
    VehicleA = vehicleA;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = globalVar(4);
    sensSamp = sensPeriod/period;
    minReward = globalVar(5);
    delL = ceil(globalVar(2)/period);
    delP = ceil(globalVar(3)/period);
    accFactor = accF;
    disFactor = disF;
    dampFactor = globalVar(10);
    delComp = globalVar(8);
    extraDis = globalVar(9);
    st = globalVar(11);
    
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
    
    VCDeComp = DelayComp(VehicleC,1,1);
    VDDeComp = DelayComp(VehicleD,1,1);
    VEDeComp = DelayComp(VehicleE,1,1);
    VFDeComp = DelayComp(VehicleF,1,1);
    VGDeComp = DelayComp(VehicleG,1,1);
    %{
    maxDisC = 0;
    maxDisD = 0;
    maxDisE = 0;
    maxDisF = 0;
    maxDisG = 0;
    %}
    vehBAcc = 0;
    vehCAcc = 0;
    vehDAcc = 0;
    vehEAcc = 0;
    vehFAcc = 0;
    vehGAcc = 0;
    %{
    vehPDisC = 10;
    vehPDisD = 10;
    vehPDisE = 10;
    vehPDisF = 10;
    vehPDisG = 10;
    
    maxAcc = 2;
    minAcc = -4.5;
    %}
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
            vehBAcc = VBCont.getAcc(0,vehPDisB,0,vehPVelB,a); 
        end
        if (a > delP) && (a > delL)
            
            vehPDisC = VehicleB.pos(a-delP) - VehicleC.pos(a-delP) -5 ;
            vehPDisD = VehicleC.pos(a-delP) - VehicleD.pos(a-delP) -5 ;
            vehPDisE = VehicleD.pos(a-delP) - VehicleE.pos(a-delP) -5 ;
            vehPDisF = VehicleE.pos(a-delP) - VehicleF.pos(a-delP) -5 ;
            vehPDisG = VehicleF.pos(a-delP) - VehicleG.pos(a-delP) -5 ;
            %%Crash
            if vehPDisC <= 0 || vehPDisD <= 0 || vehPDisE <= 0 || vehPDisF <= 0 || vehPDisG <= 0
                reward = minReward;
                %fprintf("Crashed\n");
                return
            end
            %%Sensor New Data
            if  not(mod(a,sensSamp)) %|| not(mod((a-(sensSamp/2)^(1-delComp)),sensSamp)) )
                state = 2;
                %{
                if not(mod(a,sensSamp))
                    state = 0;
                else
                    state = 1;
                    %fprintf("a = %d\n",a);
                end
                %}
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
                
                if delComp
                    [vehPVelC,vehPDisC] = VCDeComp.get(vehPVelC,vehPDisC,a,state,VehicleB.acc(a-delL));
                    [vehPVelD,vehPDisD] = VDDeComp.get(vehPVelD,vehPDisD,a,state,VehicleC.acc(a-delL));
                    [vehPVelE,vehPDisE] = VEDeComp.get(vehPVelE,vehPDisE,a,state,VehicleD.acc(a-delL));
                    [vehPVelF,vehPDisF] = VFDeComp.get(vehPVelF,vehPDisF,a,state,VehicleE.acc(a-delL));
                    [vehPVelG,vehPDisG] = VGDeComp.get(vehPVelG,vehPDisG,a,state,VehicleF.acc(a-delL));
                    
                    [vehLVelC,vehLDisC] = VGDeComp.get(vehLVelC,vehLDisC,a,state,VehicleA.acc(a-delL));
                    [vehLVelD,vehLDisD] = VGDeComp.get(vehLVelD,vehLDisD,a,state,VehicleB.acc(a-delL));
                    [vehLVelE,vehLDisE] = VGDeComp.get(vehLVelE,vehLDisE,a,state,VehicleC.acc(a-delL));
                    [vehLVelF,vehLDisF] = VGDeComp.get(vehLVelF,vehLDisF,a,state,VehicleD.acc(a-delL));
                    [vehLVelG,vehLDisG] = VGDeComp.get(vehLVelG,vehLDisG,a,state,VehicleE.acc(a-delL));
                    
                end
                
                if (~delComp) || state
                    %disp(state);
                    vehCAcc = VCCont.getAcc(vehLDisC,vehPDisC-extraDis,vehLVelC,vehPVelC-extraDis,a); 
                    vehDAcc = VDCont.getAcc(vehLDisD,vehPDisD-extraDis,vehLVelD,vehPVelD-extraDis,a);
                    vehEAcc = VECont.getAcc(vehLDisE,vehPDisE-extraDis,vehLVelE,vehPVelE-extraDis,a);
                    vehFAcc = VFCont.getAcc(vehLDisF,vehPDisF-extraDis,vehLVelF,vehPVelF-extraDis,a);
                    vehGAcc = VGCont.getAcc(vehLDisG,vehPDisG-extraDis,vehLVelG,vehPVelG-extraDis,a);
                    %{
                    PvehLVelD(count) = vehLVelD - (VehicleB.vel(a) - VehicleD.vel(a));
                    PvehLPosD(count) = vehLDisD - (VehicleB.pos(a) - VehicleD.pos(a));
                    PvehLVelG(count) = vehLVelG - (VehicleE.vel(a) - VehicleG.vel(a));
                    PvehLPosG(count) = vehLDisG - (VehicleE.pos(a) - VehicleG.pos(a));
                    count = count + 1;
                    %}
                end
                
            end
        end
        %{
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
        %}
        %%Condition check
        if a > st/period

            %if VehicleG.acc(a-2) > maxAcc 
            %    maxAcc = VehicleG.acc(a-2);
            %elseif VehicleG.acc(a-2) < minAcc
            %    minAcc = VehicleG.acc(a-2);
            %end
            
            if (VehicleG.acc(a-2)-VehicleG.acc(a-3))*((-1)^dampCount) > 0
                dampCount = dampCount + 1;
            end
            
            %reward =  -((maxDisC * maxDisD * maxDisE * maxDisF * maxDisG)^ disFactor) * ...
            %            dampCount * ...
            %            accFactor^(maxAcc / 2.5 - minAcc / 4.5);
            
        end
    end
    
    maxAcc = max(VehicleG.acc(st/period:end-2));
    minAcc = min(VehicleG.acc(st/period:end-2));
    %{
    if maxAcc < 2.5
        maxAcc = 2.5;  
    end
    if minAcc > -4.5
        minAcc = -4.5;  
    end
    %}
    maxDisC = max(VehicleB.pos-VehicleC.pos);
    maxDisD = max(VehicleC.pos-VehicleD.pos);
    maxDisE = max(VehicleD.pos-VehicleE.pos);
    maxDisF = max(VehicleE.pos-VehicleF.pos);
    maxDisG = max(VehicleF.pos-VehicleG.pos);
    reward =  -((maxDisC * maxDisD * maxDisE * maxDisF * maxDisG)^ disFactor) * ...
                        dampCount^dampFactor * ...
                        accFactor^( 1 + abs(maxAcc-2.5) + abs(minAcc + 4.5));
    %{
    close all
    figure
    hold on
    plot(PvehLVelD(delL+1:end))
    plot(PvehLVelG(delL+1:end))
    title('vel diff');
    xlabel('time(s)');
    ylabel('vel(ms-1)');
    legend({'C','G'},'Location','southeast');
    hold off

    figure
    hold on
    plot(VehicleC.acc(delL+1:end))
    plot(VehicleG.acc(delL+1:end))
    title('acc');
    xlabel('time(s)');
    ylabel('vel(ms-1)');
    legend({'C','G'},'Location','southeast');
    hold off
    
    figure
    hold on
    plot(PvehLPosD)
    plot(PvehLPosG)
    title('pos diff');
    xlabel('time(s)');
    ylabel('pos(m)');
    legend({'C','G'},'Location','southeast');
    hold off

    figure
    hold on
    plot((VehicleB.vel(delL+1:end) - VehicleD.vel(delL+1:end))-(VehicleB.vel(1:end-delL) - VehicleD.vel(1:end-delL)))
    plot((VehicleE.vel(delL+1:end) - VehicleG.vel(delL+1:end))-(VehicleE.vel(1:end-delL) - VehicleG.vel(1:end-delL)))
    title('vel diff wo');
    xlabel('time(s)');
    ylabel('pos(m)');
    legend({'C','G'},'Location','southeast');
    hold off
    
    figure
    hold on
    plot((VehicleB.pos(delL+1:end) - VehicleD.pos(delL+1:end))-(VehicleB.pos(1:end-delL) - VehicleD.pos(1:end-delL)))
    plot((VehicleE.pos(delL+1:end) - VehicleG.pos(delL+1:end))-(VehicleE.pos(1:end-delL) - VehicleG.pos(1:end-delL)))
    title('pos diff wo');
    xlabel('time(s)');
    ylabel('pos(m)');
    legend({'C','G'},'Location','southeast');
    hold off
    
    fprintf("Done\n")
    %}
end