function distance = delayCompensator(vehDis,vehVel,delInt,k1,k2)
    % Inputs:
    %   vehDis  :Delayed Sensing distance (Vector)
    %   vehVel  :Gradient of vehDis (Vector)
    %   delInt  :Delay Interval     (Scalar)
    %   k1 & 2  :Constant for calculation    (Scalar)
    if(length(vehDis) < 4)  %3 data are required for prediction
        distance =  vehDis;
    else
        vehAcc = gradient(vehVel(end-3:end));
        distance = vehDis(end) + vehVel(end-1) * delInt / k1 + ...
            vehAcc(end-2) * delInt ^ 2 / k2 ; 
    end
end