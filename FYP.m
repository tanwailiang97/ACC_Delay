%%D:\Mega\Programming\
%%D:\WL\UTM\FYP
%%C:\Users\Liang\Desktop\Programming
%clear

load Vehicle.mat VehicleA

tic
%totalTime = globalVar(0);% time in second
%period = globalVar(1);  %sampling period
%sensPeriod = globalVar(4);

fprintf("%s\nStarting FYP ...\n",datetime('now','Format','y-MMM-d HH-mm-ss'));

maxParallel = 40;    

parfor parallel = 1:maxParallel
    minReward = globalVar(5);
    fprintf("Starting Parallel...%d\n",parallel);
    maxWorker = 20;
    param = [0 0 0 0 0 0];
    prevResult = minReward;
    step = 0.001*2^10;
    prevWorker = maxWorker + 2;
    bestWorker = maxWorker + 2;
    flag = 0;
    newParam =  zeros(maxWorker,6);
    result = zeros(maxWorker,1);
    while(1)
        for worker = 1:maxWorker
            %fprintf("Running Worker %d\n",worker);
            if worker > 12 || prevResult == minReward
                newParam(worker,:) = (randi(16,1,6)-8);
                result(worker) = vehicleRunning(newParam(worker,:),VehicleA);
            else
                identity = eye(6);
                newParam(worker,:) = param + (-1)^(worker) * step *...
                               identity(idivide(worker+1,int8(2)),:);
                result(worker) = vehicleRunning(newParam(worker,:),VehicleA);
            end
        end
        bestWorkerOld = bestWorker;
        [maxResult,bestWorker] = max(result);
        if maxResult ~= minReward
            if maxResult > prevResult
                prevResult = maxResult;
                prevWorker = bestWorker;
                param = newParam(bestWorker,:);
                fprintf("%.2f,\t",param);
                fprintf("%.2f,\n",maxResult);
                flag = 1;
            elseif (idivide(bestWorker-1,int8(2)) == idivide(prevWorker-1,int8(2)) ||...
                (bestWorkerOld == bestWorker && bestWorker < maxWorker + 2)) ...
                && flag == 1
                step = step/2;
                if step <= 0.001
                    %fprintf("low step = %f\n",step);
                    break
                end
                fprintf("step = %f\n",step);
            else 
                fprintf("Worker %d,%d\n",bestWorker,prevWorker);
            end
        else
            
        end

    end
 
fprintf("%.2f,\t",param);
resultSave(param,prevResult,parallel,VehicleA);

disp('done')
fprintf('time:%f\n',toc)

    %for beepCount = 1:3
    %    beep
    %    pause(1)
    %end
end

%for beepCount = 1:20
%        beep
%        pause(1)
%end


