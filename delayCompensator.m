function distance = delayCompensator(vehDis,vehVel,delInt,sF1,sF2)
    if(length(vehDis) < 4) %3 data are required for prediction
        distance =  vehDis;
    else
        vehAcc = gradient(vehVel(end-3:end));
        distance = vehDis(end) + vehVel(end-1) * delInt / sF1 + vehAcc(end-2) * delInt ^ 2 / sF2 ; 
    end
end