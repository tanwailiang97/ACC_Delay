totalTime = globalVar(0);% time in second
period = globalVar(1);  %sampling period

VehicleA = Vehicle(1055,2.306,0.01,1.962,-1.962,0,30);
VehicleB = Vehicle(1055,2.306,0.01,1.962,-1.962,0,25);

desVelInc = 10;
desVelDec = -5;

param = [8.8*period*5 , 0.54 , 1.08 , -0.02];%wn,zeta,peak,offset
steps = 0.001*2^12;
bestWorker = 9;
prevResult = -Inf;

while true
    
    for worker = 1:9
        if worker == 9
            paramTemp = param;
            newParam(worker,:) = paramTemp;
        else
            identity = eye(4);
            paramTemp = param + (-1)^(worker) * steps *...
                                identity(idivide(worker+1,int8(2)),:);
            newParam(worker,:) = paramTemp;
        end
        wn   = paramTemp(1);
        zeta = paramTemp(2);
        peak = paramTemp(3);
        offset = paramTemp(4);
        sys1 = tf(peak*wn^2,[1,2*zeta*wn,wn^2]);
        sys1Res = step(sys1,0:period:40-period);
        jerkRising = peak - sys1Res + offset;
        acc = cumtrapz(jerkRising)*period;
        vel = cumtrapz(acc)*period;
        result(worker) = -(abs(acc(end))* abs(vel(end)-10));
    end
    bestWorkerOld = bestWorker;
    [maxResult,bestWorker] = max(result);
    
    if maxResult > prevResult
        prevResult = maxResult;
        prevWorker = bestWorker;
        param = newParam(bestWorker,:);
        fprintf("%.3f,",param);
        fprintf(" > %g\n",maxResult);
    elseif ((idivide(bestWorker-1,int8(2)) == idivide(prevWorker-1,int8(2))) ... 
            || (bestWorkerOld == bestWorker))
        steps = steps/2;
        fprintf("step = %f\n",steps);
    elseif  steps <= 0.000001 %|| maxResult == prevResult
        fprintf("ended");
        break
    else
        fprintf("stucked");
    end

end

wn   = param(1);
zeta = param(2);
peak = param(3);
offset = param(4);
sys1 = tf(peak*wn^2,[1,2*zeta*wn,wn^2]);
sys1Res = step(sys1,0:period:40-period);
jerkRising = peak - sys1Res + offset;
acc = cumtrapz(jerkRising)*period;
vel = cumtrapz(acc)*period;

close all
figure
plot(jerkRising);
figure
plot(acc);
figure
plot(vel);



%{
VehicleA.acc = cumtrapz(VehicleA.jerk)*VehicleA.period ;
VehicleA.vel = cumtrapz(VehicleA.acc)*VehicleA.period ;
VehicleA.pos = cumtrapz(VehicleA.vel)*VehicleA.period + VehicleA.offPos ;


%{
for x = 1: (totalTime/period)
    k = -1/4.5;         %38s
    k2 = -1/(4.5/19);   %2s
    j1 = 1.1;
    j2 = -0.5;
    y = x * period;
    if y <= 2
        %VehicleA.acc(x) = (-10*k) * (1 - exp(k2*y));
        VehicleA.jerk(x) = 10*(-k*exp(k*y));
    elseif y <= 40
        VehicleA.acc(x) = 10*(-k*exp(k*y));
    elseif y <= 80
        VehicleA.acc(x) = 10*(-k*exp(k*y)*exp(-40*k));
    elseif y <= 120 
        VehicleA.acc(x) = -5*(-k*exp(k*y)*exp(-80*k));
    elseif y <= 160
        VehicleA.acc(x) = 10*(-k*exp(k*y)*exp(-120*k));
    end
end
%}
%{
for x = 1: (totalTime/period)
    k = -1/6.86;
    y = x * period;
    if(y <= 40)
        VehicleA.vel(x) = 10*(1-exp(k*y));
    elseif (y <= 80) 
        VehicleA.vel(x) = VehicleA.vel(40/period) + 10*(1-exp(k*y)*exp(-40*k));
    elseif (y <= 120) 
        VehicleA.vel(x) = VehicleA.vel(80/period) - 5*(1-exp(k*y)*exp(-80*k));
    elseif (y <= 160) 
        VehicleA.vel(x) = VehicleA.vel(120/period) + 10*(1-exp(k*y)*exp(-120*k));
    end
end
%}

%VehicleA.acc = gradient(VehicleA.vel)/VehicleA.period;
%VehicleA.jerk = del2(VehicleA.vel,VehicleA.period);



VehicleB.pos = VehicleA.pos - VehicleA.vel*0.1 - 5;
VehicleB.vel = gradient(VehicleB.pos)/VehicleB.period;
VehicleB.acc = del2(VehicleB.pos,VehicleB.period);


timePlot = globalVar(1):globalVar(1):(totalTime);

close all

figure;
hold on
plot(timePlot,VehicleA.jerk);
title("Jerk");
%plot(timePlot,VehicleB.acc);
%legend({'A','B'},'Location','southeast');
hold off

figure;
hold on
plot(timePlot,VehicleA.acc);
title("Acc");
%plot(timePlot,VehicleB.acc);
%legend({'A','B'},'Location','southeast');
hold off

figure;
hold on
plot(timePlot,VehicleA.vel);
%plot(timePlot,VehicleB.vel);
title("Vel");
%legend({'A','B'},'Location','southeast');
hold off

figure;
hold on
plot(timePlot,VehicleA.pos-VehicleB.pos);
title("Pos");
%plot(timePlot,VehicleB.pos);
%legend({'A','B'},'Location','southeast');
hold off
%}
