function resultSave(param,result,parallel,vehicleA,vehicleB)
    %load Vehicle.mat VehicleA
    VehicleA = vehicleA;
    VehicleB = vehicleB;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = globalVar(4);
    sensSamp = sensPeriod/period;
    delL = ceil(globalVar(2)/period);
    delP = ceil(globalVar(3)/period);
    delComp = globalVar(8);
    extraDis = globalVar(9);
    sens2 = globalVar(13);
    tao = globalVar(14);
    dampCount = 1;
    
    st = globalVar(11);
    
    u = param(1);
    v = param(2);
    w = param(3);
    x = param(4);
    y = param(5);
    z = param(6);
    
    %VehicleB = Vehicle(1055,3,0,3,-6,5,23);
    VehicleC = Vehicle(1055,3,tao,2,-2,0,20);
    VehicleD = Vehicle(1055,3,tao,2,-2,0,15);
    VehicleE = Vehicle(1055,3,tao,2,-2,0,10);
    VehicleF = Vehicle(1055,3,tao,2,-2,0,5);
    VehicleG = Vehicle(1055,3,tao,2,-2,0,0);
    
    %VBCont = AccController(VehicleB,7.92,-0.352,2.96,8.72,0,0);
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

    %vehBAcc = 0;
    vehCAcc = 0;
    vehDAcc = 0;
    vehEAcc = 0;
    vehFAcc = 0;
    vehGAcc = 0;
    
    for a = 1:(totalTime/period)

        %VehicleB.move(vehBAcc);
        VehicleC.move(vehCAcc,a);
        VehicleD.move(vehDAcc,a);
        VehicleE.move(vehEAcc,a);
        VehicleF.move(vehFAcc,a);
        VehicleG.move(vehGAcc,a);
        
        %if a > 1
            %vehPDisB = VehicleA.pos(a-1) - VehicleB.pos(a-1) -5 ;
            %vehPVelB = VehicleA.vel(a-1) - VehicleB.vel(a-1);
            %vehBAcc  = VBCont.getAcc(0,vehPDisB,0,vehPVelB,a); 
        %end
        if (a > delP) && (a > delL)
            
            vehPDisC = VehicleB.pos(a-delP) - VehicleC.pos(a-delP) -3 ;
            vehPDisD = VehicleC.pos(a-delP) - VehicleD.pos(a-delP) -3 ;
            vehPDisE = VehicleD.pos(a-delP) - VehicleE.pos(a-delP) -3 ;
            vehPDisF = VehicleE.pos(a-delP) - VehicleF.pos(a-delP) -3 ;
            vehPDisG = VehicleF.pos(a-delP) - VehicleG.pos(a-delP) -3 ;
            %%Sensor New Data
            if  (not(mod(a,sensSamp))) || (sens2 && not(mod((a-(sensSamp/2)^(1-delComp)),sensSamp)))
                if sens2
                    if not(mod(a,sensSamp))
                        state = 0;
                    else
                        state = 1;
                    end
                else
                    state = 2;
                end
                
                %}
                vehPVelC = VehicleB.vel(a-delP) - VehicleC.vel(a-delP);
                vehPVelD = VehicleC.vel(a-delP) - VehicleD.vel(a-delP);
                vehPVelE = VehicleD.vel(a-delP) - VehicleE.vel(a-delP);
                vehPVelF = VehicleE.vel(a-delP) - VehicleF.vel(a-delP);
                vehPVelG = VehicleF.vel(a-delP) - VehicleG.vel(a-delP);

                vehLDisC = VehicleA.pos(a-delL) - VehicleC.pos(a-delL) -6 ;
                vehLDisD = VehicleB.pos(a-delL) - VehicleD.pos(a-delL) -6 ;
                vehLDisE = VehicleC.pos(a-delL) - VehicleE.pos(a-delL) -6 ;
                vehLDisF = VehicleD.pos(a-delL) - VehicleF.pos(a-delL) -6 ;
                vehLDisG = VehicleE.pos(a-delL) - VehicleG.pos(a-delL) -6 ;

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
                    vehCAcc = VCCont.getAcc(vehLDisC-2*extraDis,vehPDisC-extraDis,vehLVelC,vehPVelC,a); 
                    vehDAcc = VDCont.getAcc(vehLDisD-2*extraDis,vehPDisD-extraDis,vehLVelD,vehPVelD,a);
                    vehEAcc = VECont.getAcc(vehLDisE-2*extraDis,vehPDisE-extraDis,vehLVelE,vehPVelE,a);
                    vehFAcc = VFCont.getAcc(vehLDisF-2*extraDis,vehPDisF-extraDis,vehLVelF,vehPVelF,a);
                    vehGAcc = VGCont.getAcc(vehLDisG-2*extraDis,vehPDisG-extraDis,vehLVelG,vehPVelG,a);
                end
                %%Condition check
            
            end
            if a > st/period
                if (VehicleG.acc(end)-VehicleG.acc(end-1))*((-1)^dampCount) > 0
                    dampCount = dampCount + 1;
                end
            end
        end
    end
    

    timePlot = 0:period:(totalTime-st);
    date = datetime('now','Format','y-MMM-d');
    time = datetime('now','Format','HH-mm-ss');
    fileLoc = sprintf('FYP/Image/%s/%d-%f %d %f %s-%d/',...
        date,globalVar(8),globalVar(2),dampCount...
        ,max(VehicleG.acc),time,parallel);
    
    mkdir(fileLoc)
    txtName = sprintf('%sparam.txt',fileLoc);
    fid = fopen(txtName,'w');
    fprintf(fid,"%.3f,",param);
    fprintf(fid,"\nResult\t: %.20g",result);
    fprintf(fid,"\nDelay\t: %f,%f",globalVar(2),globalVar(3));
    fprintf(fid,"\nTao\t: %f",globalVar(14));
    fprintf(fid,"\nPeriod\t: %f",globalVar(4));
    fprintf(fid,"\nAcc Factor\t: %f",globalVar(6));
    fprintf(fid,"\nDis Factor\t: %f",globalVar(7));
    fprintf(fid,"\nDamp Factor\t: %f",globalVar(10));
    fprintf(fid,"\nDelay Compensated\t: %f",globalVar(8));
    fprintf(fid,"\nExtra Dis\t: %f",globalVar(9));
    fprintf(fid,"\nSensor 2\t: %f",globalVar(13));
    fprintf(fid,"\nDamp Count\t: %d",dampCount);
    fprintf(fid,"\nMax Distance\t: %f",max(VehicleA.pos-VehicleG.pos));
    
    fclose(fid);
    %writematrix(param,txtName);
    
    
    %close all
    
    fig1 = figure;
    hold on
    %plot(timePlot,VehicleA.acc((st/period:(totalTime-ed)/period)));
    %plot(timePlot,VehicleB.acc((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleC.acc(st/period:totalTime/period));
    plot(timePlot,VehicleD.acc(st/period:totalTime/period));
    plot(timePlot,VehicleE.acc(st/period:totalTime/period));
    plot(timePlot,VehicleF.acc(st/period:totalTime/period));
    plot(timePlot,VehicleG.acc(st/period:totalTime/period));
    title('accleration');
    xlabel('time(s)');
    ylabel('acceleration(ms-2)');
    legend({'A','B','C','D','E'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig1',fileLoc);
    print(fig1,'-djpeg','-r1000',imageName);
    close(fig1);
    
    

    fig2 = figure;
    hold on
    %plot(timePlot,VehicleA.vel((st/period:(totalTime-ed)/period)));
    %plot(timePlot,VehicleB.vel((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleC.vel(st/period:totalTime/period));
    plot(timePlot,VehicleD.vel(st/period:totalTime/period));
    plot(timePlot,VehicleE.vel(st/period:totalTime/period));
    plot(timePlot,VehicleF.vel(st/period:totalTime/period));
    plot(timePlot,VehicleG.vel(st/period:totalTime/period));
    title('velocity');
    xlabel('time(s)');
    ylabel('velocity(ms-1)');
    legend({'A','B','C','D','E'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig2',fileLoc);
    print(fig2,'-djpeg','-r1000',imageName);
    close(fig2);
    %{
    fig3 = figure;
    hold on
    plot(timePlot,VehicleA.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleB.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleC.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleD.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleE.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleF.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleG.pos((st/period:(totalTime-ed)/period)));
    title('position');
    xlabel('time(s)');
    ylabel('position(m)');
    legend({'A','B','C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig3',fileLoc);
    print(fig3,'-djpeg','-r1000',imageName);
    close(fig3);

%}
    fig4 = figure;
    hold on
    %plot(timePlot,VehicleA.pos((st/period:(totalTime-ed)/period))...
    %            -VehicleB.pos((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleB.pos(st/period:totalTime/period)...
                -VehicleC.pos(st/period:totalTime/period));
    plot(timePlot,VehicleB.pos(st/period:totalTime/period)...
                -VehicleD.pos(st/period:totalTime/period));
    plot(timePlot,VehicleB.pos(st/period:totalTime/period)...
                -VehicleE.pos(st/period:totalTime/period));
    plot(timePlot,VehicleB.pos(st/period:totalTime/period)...
                -VehicleF.pos(st/period:totalTime/period));
    plot(timePlot,VehicleB.pos(st/period:totalTime/period)...
                -VehicleG.pos(st/period:totalTime/period));
    title('position different');
    xlabel('time(s)');
    ylabel('position(m)');
    legend({'A','B','C','D','E'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig4',fileLoc);
    print(fig4,'-djpeg','-r1000',imageName);
    close(fig4);
%{
    fig5 = figure;
    hold on
    %plot(timePlot,VehicleA.acc((st/period:(totalTime-ed)/period))...
    %            -VehicleB.acc((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleB.acc((st/period:(totalTime-ed)/period))...
                -VehicleC.acc((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleC.acc((st/period:(totalTime-ed)/period))...
                -VehicleD.acc((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleD.acc((st/period:(totalTime-ed)/period))...
                -VehicleE.acc((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleE.acc((st/period:(totalTime-ed)/period))...
                -VehicleF.acc((st/period:(totalTime-ed)/period)));
    plot(timePlot,VehicleF.acc((st/period:(totalTime-ed)/period))...
                -VehicleG.acc((st/period:(totalTime-ed)/period)));
    title('accleration different');
    xlabel('time(s)');
    ylabel('acceleration(ms-2)');
    legend({'C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig5',fileLoc);
    print(fig5,'-djpeg','-r1000',imageName);
    close(fig5);
    
%}
    
    %close([fig1 fig2 fig3 fig4 fig5]);
end