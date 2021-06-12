function [VehC,VehD] = resultSave(param,result,parallel,vehicleA,vehicleB,sensD)
    VehicleA = vehicleA;
    VehicleB = vehicleB;
    totalTime = globalVar(0);% time in second
    period = globalVar(1);  %sampling period
    sensPeriod = sensD;
    sensSamp = round(sensPeriod/period);
    delL = round(sensD/period);
    delP = round(sensD/period);
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

    VehicleC = Vehicle(1055,3,tao,2,-2,0,20);
    VehicleD = Vehicle(1055,3,tao,2,-2,0,15);
    
    VCCont = AccController(VehicleC,u,v,w,x,y,z);
    VDCont = AccController(VehicleD,u,v,w,x,y,z);
    
    VCDeComp = DelayComp(VehicleC,1,1,sensD);
    VDDeComp = DelayComp(VehicleD,1,1,sensD);

    vehCAcc = 0;
    vehDAcc = 0;
    
    for a = 1:(totalTime/period)

        VehicleC.move(vehCAcc,a);
        VehicleD.move(vehDAcc,a);
        
        if (a > delP) && (a > delL)
            
            vehPDisC = VehicleB.pos(a-delP) - VehicleC.pos(a-delP) -3 ;
            vehPDisD = VehicleC.pos(a-delP) - VehicleD.pos(a-delP) -3 ;
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
            if a > st/period
                if (VehicleD.acc(end)-VehicleD.acc(end-1))*((-1)^dampCount) > 0
                    dampCount = dampCount + 1;
                end
            end
        end
    end
    
    timePlot = 0:period:(totalTime-st);
    date = datetime('now','Format','y-MMM-d');
    time = datetime('now','Format','HH-mm-ss');
    fileLoc = sprintf('FYP/Image/%s/%d-%f %f %f %s-%d/',...
        date,globalVar(8),sensD,max(VehicleA.pos-VehicleD.pos)...
        ,max([VehicleD.acc VehicleC.acc]),time,parallel);
    
    mkdir(fileLoc)
    txtName = sprintf('%sparam.txt',fileLoc);
    fid = fopen(txtName,'w');
    fprintf(fid,"%.3f,",param);
    fprintf(fid,"\nResult\t: %.20g",result);
    fprintf(fid,"\nDelay\t: %f,%f",sensD,sensD);
    fprintf(fid,"\nTao\t: %f",globalVar(14));
    fprintf(fid,"\nPeriod\t: %f",sensD);
    fprintf(fid,"\nAcc Factor\t: %f",globalVar(6));
    fprintf(fid,"\nDis Factor\t: %f",globalVar(7));
    fprintf(fid,"\nDamp Factor\t: %f",globalVar(10));
    fprintf(fid,"\nDelay Compensated\t: %f",globalVar(8));
    fprintf(fid,"\nExtra Dis\t: %f",globalVar(9));
    fprintf(fid,"\nSensor 2\t: %f",globalVar(13));
    fprintf(fid,"\nDamp Count\t: %d",dampCount);
    fprintf(fid,"\nMax Distance\t: %f",max(VehicleA.pos-VehicleD.pos));
    fprintf(fid,"\nAvg Max CD Distance\t: %f",0.5*(max(VehicleC.pos-VehicleD.pos)+max(VehicleB.pos-VehicleC.pos)));
    
    fclose(fid);
    
    fig1 = figure;
    hold on
    plot(timePlot,VehicleA.acc(st/period:totalTime/period));
    plot(timePlot,VehicleB.acc(st/period:totalTime/period));
    plot(timePlot,VehicleC.acc(st/period:totalTime/period));
    plot(timePlot,VehicleD.acc(st/period:totalTime/period));
    title('Accleration');
    xlabel('time(s)');
    ylabel('acceleration(ms-2)');
    legend({'A','B','C','D'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig1',fileLoc);
    print(fig1,'-djpeg','-r1000',imageName);
    close(fig1);

    fig2 = figure;
    hold on
    plot(timePlot,VehicleA.vel(st/period:totalTime/period));
    plot(timePlot,VehicleB.vel(st/period:totalTime/period));
    plot(timePlot,VehicleC.vel(st/period:totalTime/period));
    plot(timePlot,VehicleD.vel(st/period:totalTime/period));
    title('Velocity');
    xlabel('time(s)');
    ylabel('velocity(ms-1)');
    legend({'A','B','C','D'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig2',fileLoc);
    print(fig2,'-djpeg','-r1000',imageName);
    close(fig2);
    
    fig3 = figure;
    hold on
    plot(timePlot,gradient(VehicleA.acc(st/period:totalTime/period)));
    plot(timePlot,gradient(VehicleB.acc(st/period:totalTime/period)));
    plot(timePlot,gradient(VehicleC.acc(st/period:totalTime/period)));
    plot(timePlot,gradient(VehicleD.acc(st/period:totalTime/period)));
    title('Jerk');
    xlabel('time(s)');
    ylabel('Jerk(ms-3)');
    legend({'A','B','C','D'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig3',fileLoc);
    print(fig3,'-djpeg','-r1000',imageName);
    close(fig3);
   
    fig4 = figure;
    hold on
    plot(timePlot,VehicleA.pos(st/period:totalTime/period)...
                -VehicleB.pos(st/period:totalTime/period));
    plot(timePlot,VehicleA.pos(st/period:totalTime/period)...
                -VehicleC.pos(st/period:totalTime/period));
    plot(timePlot,VehicleA.pos(st/period:totalTime/period)...
                -VehicleD.pos(st/period:totalTime/period));
    title('position different');
    xlabel('time(s)');
    ylabel('position(m)');
    legend({'B','C','D'},'Location','southeast');
    hold off
    imageName = sprintf('%sfig4',fileLoc);
    print(fig4,'-djpeg','-r1000',imageName);
    close(fig4);
   
    VehC = VehicleC;
    VehD = VehicleD;
end