clear

dI = 8;
Ts = 0.1;           %Sampling Interval
accRatio = 9.81;
sample = 200;

aA = zeros(sample,1);   %Acceleration of A
vA = zeros(sample,2);   %Velocity of A
pA = zeros(sample,3);   %Position of A , delayed , predicted
%diff = zeros(sample-dI,2);   %Position of A , delayed , predicted


for x = 2:sample
    if x > sample*2/4
        aA(x) = -accRatio ;%+ (4)*rand();
    %elseif x > sample /4 && x <= sample*3.5/4
    %    aA(x) = 0;
    %elseif (vA(x-1) < 100)
    %    aA(x) = accRatio ;%- (4)*rand();
    else
        %aA(x) = 0;
        aA(x) = accRatio;
    end
    vA(x,1) = (aA(x-1)+aA(x))/2*Ts + vA(x-1,1);
    pA(x,1) = (vA(x-1,1)+vA(x,1))/2*Ts + pA(x-1,1);
end

for x = 2+dI:length(pA(:,1))
    pA(x,2) = pA(x-dI,1);
    %pA(x,3) = pA(x,2);
    %pA(x,3) = pA(x,2) + (pA(x-dI,1)-pA(x-dI-1,1))*(dI-0.5);
    pA(x,3) = delayCompensator(pA(1:x,2),gradient(pA(1:x,2)),dI,1,1.25);
end




%diff(:,1) = (pA(2+dI:sample,1)- pA(2+dI:sample,2));
%diff(:,2) = (pA(2+dI:sample,1)- pA(2+dI:sample,3));
diff(:,1) = abs(pA(:,1)- pA(:,2));
diff(:,2) = abs(pA(:,1)- pA(:,3));
maxDiff(1) = max(diff(:,1));
maxDiff(2) = max(diff(:,2));

maxDiff

%disp(maxDiff)
%disp(mean(maxDiff,1));
%disp(mean(maxDiff,2));
tA = 0:Ts:Ts*(sample-1);

%[ax,hlines,fh] = jzplotys(xy_pairs,ax_groups,xrange)
%[ax,hlines,fh] = jzplotys({tA,pA(:,1),tA,pA(:,2),tA,pA(:,3),tA,vA(:,1),tA,aA},[3 1 1],[0 tA(end)],100);
%legend(cat(1,hlines{:}),'Real Time Data','Delayed Data','Predicted Data','Velocity','Accleration','location',[.88 .5 .01 .01]);
[ax,hlines,fh] = jzplotys({tA,diff(:,1),tA,diff(:,2),tA,vA(:,1),tA,aA},[2 1 1],[0 tA(end)],100);
legend(cat(1,hlines{:}),'Delayed Data','Predicted Data','Velocity','Accleration','location',[.88 .5 .01 .01]);
xlabel(ax(1),'Time(s)');
ylabel(ax(1),'Difference compare with real time data(m)');
ylabel(ax(3),'Velocity(m/s)');
ylabel(ax(5),'Accleration(m/s^2)');
hold off



