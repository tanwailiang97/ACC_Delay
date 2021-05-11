function x = globalVar(num)
    TOTAL_TIME = 60;    %total Time for simulation
    PERIOD = 0.01;      %10ms global sampling 
    SENS_DELAY_1 = 0.2; %200ms sensor delay for Leader
    SENS_DELAY_2 = 0.2; %200ms sensor delay for Preceder
    SENS_PERIOD = 0.06; %60ms update rate
    PREV_RESULT = -780000;
    if num == 0
        x = TOTAL_TIME;
    elseif num == 1
        x = PERIOD;
    elseif num == 2
        x = SENS_DELAY_1;
    elseif num == 3
        x = SENS_DELAY_2;
    elseif num == 4
        x = SENS_PERIOD;
    elseif num == 5
        x = PREV_RESULT;
    end
end