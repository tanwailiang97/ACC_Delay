function x = globalVar(num)
    PERIOD = 0.01; %10ms global sampling 
    SENS_DELAY_1 = 0.2; %200ms sensor delay for Leader
    SENS_DELAY_2 = 0.2; %200ms sensor delay for Preceder
    
    
    if num == 1
        x = PERIOD;
    elseif num == 2
        x = SENS_DELAY_1;
    elseif num == 3
        x = SENS_DELAY_2;
    end
end