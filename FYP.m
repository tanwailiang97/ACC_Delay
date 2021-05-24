%%D:\Mega\Programming\
%%D:\WL\UTM\
%%C:\Users\Liang\Desktop\Programming
%clear

addpath(genpath('D:/WL/UTM/FYP'))

load Vehicle.mat VehicleA

tic

fprintf("%s\nStarting FYP ...\n",datetime('now','Format','y-MMM-d HH-mm-ss'));

maxParallel = 40;    

parfor parallel = 1:maxParallel
    
    minReward = globalVar(5);
    accFactor = globalVar(6);
    disFactor = globalVar(7);
    fprintf("%s\nStarting Parallel...%d\n",datetime('now','Format','y-MMM-d HH-mm-ss'),parallel);
    maxWorker = 16;
    param = [0.000,-3.566,3.130,4.194,1.830,0.158];
    prevResult = minReward;
    step = 0.001*2^13;
    prevWorker = maxWorker + 2;
    bestWorker = maxWorker + 2;
    flag = 0;
    newParam =  zeros(maxWorker,6);
    result = ones(maxWorker,1)*minReward;
    
    while(1)
        for worker = 3:maxWorker
            %fprintf("Running Worker %d\n",worker);
            if worker > 13 || prevResult == minReward
                newParam(worker,:) = [0,(randi(1600,1,5)-800)*0.01];
            elseif worker == 13
                newParam(worker,:) = param;
            else
                identity = eye(6);
                newParam(worker,:) = param + (-1)^(worker) * step *...
                               identity(idivide(worker+1,int8(2)),:);
            end
            result(worker) = vehicleRunning(newParam(worker,:),VehicleA,accFactor,disFactor);
        end
        bestWorkerOld = bestWorker;
        [maxResult,bestWorker] = max(result);
        if maxResult ~= minReward
            if maxResult > prevResult
                prevResult = maxResult;
                prevWorker = bestWorker;
                param = newParam(bestWorker,:);
                fprintf("%d - ",parallel);
                fprintf("%.3f,",param);
                fprintf(" > %g\n",maxResult);
                flag = 1;
            elseif ((idivide(bestWorker-1,int8(2)) == idivide(prevWorker-1,int8(2)) ... 
                    && bestWorker < 13) || (bestWorkerOld == bestWorker && bestWorker < maxWorker + 2)) ...
                    && flag == 1

                step = step/2;
                if step <= 0.001
                    %fprintf("low step = %f\n",step);
                    break
                end
                fprintf("%d - step = %f\n",parallel,step);
            else 
                %fprintf("Worker %d,%d\n",bestWorker,prevWorker);
            end
        else
            %fprintf("Equal\n");
        end

    end
 
fprintf("%d - ",parallel);
fprintf("%.3f,",param);
resultSave(param,prevResult,parallel,VehicleA);

fprintf('\n%d - done\n\n',parallel)
%fprintf('time:%f\n',toc)

end



