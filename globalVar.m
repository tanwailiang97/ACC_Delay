function x = globalVar(num)
    TOTAL_TIME = 60;    %total Time for simulation
    PERIOD = 0.01;      %10ms global sampling 
    SENS_DELAY_1 = 0.3; %200ms sensor delay for Leader
    SENS_DELAY_2 = 0.3; %200ms sensor delay for Preceder
    SENS_PERIOD = 0.06; %60ms update rate
    PREV_RESULT = -18*10^15;
    ACC_FACTOR = 15;%randi(20)+5;
    DIS_FACTOR = 1;%randi(4)+1;
    DELAY_COMPENSATED  = 1;
    EXTRA_DIS = 5*DELAY_COMPENSATED;
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
    elseif num == 6
        x = ACC_FACTOR;
    elseif num == 7
        x = DIS_FACTOR;
    elseif num == 8
        x = DELAY_COMPENSATED;
    elseif num == 9
        x = EXTRA_DIS;
    end
end