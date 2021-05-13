function resultSave(param,result,parallel,vehicleA)
    %load Vehicle.mat VehicleA
    VehicleA = vehicleA;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = globalVar(4);
    delL = ceil(globalVar(2)/period);
    delP = ceil(globalVar(3)/period);
    
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

    vehBAcc = 0;
    vehCAcc = 0;
    vehDAcc = 0;
    vehEAcc = 0;
    vehFAcc = 0;
    vehGAcc = 0;
    
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
    end

    timePlot = 0:period:(totalTime-20);
    date = datetime('now','Format','y-MMM-d');
    time = datetime('now','Format','HH-mm-ss');
    fileLoc = sprintf('FYP/Image/%s/%f %f %s-%d/',...
        date,max(VehicleF.pos-VehicleG.pos)...
        ,max(VehicleG.acc((1/period:(totalTime-19)/period))),time,parallel);
    
    mkdir(fileLoc)
    txtName = sprintf('%sparam.txt',fileLoc);
    fid = fopen(txtName,'w');
    fprintf(fid,"%.3f,",param);
    fprintf(fid,"\nResult\t: %.8f",result);
    fprintf(fid,"\nDelay\t: %f,%f",globalVar(2),globalVar(3));
    fprintf(fid,"\nPeriod\t: %f",globalVar(4));
    fprintf(fid,"\nAcc Factor\t: %f",globalVar(6));
    fprintf(fid,"\nDis Factor\t: %f",globalVar(7));
    fclose(fid);
    %writematrix(param,txtName);
    
    %close all

    fig1 = figure;
    hold on
    plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleB.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleC.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleD.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleE.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleF.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleG.acc((1/period:(totalTime-19)/period)));
    title('accleration');
    xlabel('time(s)');
    ylabel('acceleration(ms-2)');
    legend({'A','B','C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig1',fileLoc);
    print(fig1,'-djpeg','-r1000',imageName);
    close(fig1);

    fig2 = figure;
    hold on
    plot(timePlot,VehicleA.vel((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleB.vel((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleC.vel((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleD.vel((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleE.vel((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleF.vel((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleG.vel((1/period:(totalTime-19)/period)));
    title('velocity');
    xlabel('time(s)');
    ylabel('velocity(ms-1)');
    legend({'A','B','C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig2',fileLoc);
    print(fig2,'-djpeg','-r1000',imageName);
    close(fig2);
    
    fig3 = figure;
    hold on
    plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleB.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleC.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleD.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleE.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleF.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleG.pos((1/period:(totalTime-19)/period)));
    title('position');
    xlabel('time(s)');
    ylabel('position(m)');
    legend({'A','B','C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig3',fileLoc);
    print(fig3,'-djpeg','-r1000',imageName);
    close(fig3);


    fig4 = figure;
    hold on
    %plot(timePlot,VehicleA.pos((1/period:(totalTime-19)/period))...
    %            -VehicleB.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleB.pos((1/period:(totalTime-19)/period))...
                -VehicleC.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleC.pos((1/period:(totalTime-19)/period))...
                -VehicleD.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleD.pos((1/period:(totalTime-19)/period))...
                -VehicleE.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleE.pos((1/period:(totalTime-19)/period))...
                -VehicleF.pos((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleF.pos((1/period:(totalTime-19)/period))...
                -VehicleG.pos((1/period:(totalTime-19)/period)));
    title('position different');
    xlabel('time(s)');
    ylabel('position(m)');
    legend({'C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig4',fileLoc);
    print(fig4,'-djpeg','-r1000',imageName);
    close(fig4);

    fig5 = figure;
    hold on
    %plot(timePlot,VehicleA.acc((1/period:(totalTime-19)/period))...
    %            -VehicleB.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleB.acc((1/period:(totalTime-19)/period))...
                -VehicleC.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleC.acc((1/period:(totalTime-19)/period))...
                -VehicleD.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleD.acc((1/period:(totalTime-19)/period))...
                -VehicleE.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleE.acc((1/period:(totalTime-19)/period))...
                -VehicleF.acc((1/period:(totalTime-19)/period)));
    plot(timePlot,VehicleF.acc((1/period:(totalTime-19)/period))...
                -VehicleG.acc((1/period:(totalTime-19)/period)));
    title('accleration different');
    xlabel('time(s)');
    ylabel('acceleration(ms-2)');
    legend({'C','D','E','F','G'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig5',fileLoc);
    print(fig5,'-djpeg','-r1000',imageName);
    close(fig5);
    %close([fig1 fig2 fig3 fig4 fig5]);
end