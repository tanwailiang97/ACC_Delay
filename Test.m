
addpath(genpath('D:/WL/UTM/FYP'))
[A,B] = VehicleGeneration(10,-5);
close all
hold on
plot(A.vel)
plot(A.pos-B.pos)
plot(A.acc)
plot(A.jerk)
hold off


%{
minN = 0;
minAcc = 999;
for n = 0.0001:0.0001:1
    k = -1/6.86;
    for x = 1:4/0.01
        y = x * 0.01;
        if y <= 2
            vel(x) = 10 * n *((y)/2)^2;
        else
            vel(x) = vel(2/0.01) + 10 * (1-n)*(1-exp(k*(y-2))); 
        end
    end
    
    acc = gradient(vel)/0.01;
    
    if max(abs(acc)) < minAcc
        minAcc = max(abs(acc));
        minN = n;
    end
end
disp(minN);


n = minN;
for x = 1:4/0.01
    y = x * 0.01;
    if y <= 2
        vel(x) = 10* n *((y/2)^2);
    else
        vel(x) = vel(2/0.01) + 10*(1-n)*(1-exp(k*(y-2))); 
    end
end
acc = gradient(vel)/0.01;
close all

hold on
plot(vel)
plot(acc)
hold off

%}