function x = globalVar(num)
    TOTAL_TIME = 170;    %total Time for simulation
    PERIOD = 0.01;      %10ms global sampling 
    SENS_DELAY_1 = 1.0; %200ms sensor delay for Leader
    SENS_DELAY_2 = SENS_DELAY_1; %200ms sensor delay for Preceder
    SENS_PERIOD = SENS_DELAY_1; %60ms update rate
    PREV_RESULT = -18*10^80;
    ACC_FACTOR = 5;%randi(20)+5;
    DIS_FACTOR = 10;%randi(4)+1;
    DELAY_COMPENSATED  = 0;
    EXTRA_DIS = 2;
    DAMP_FACTOR = 20;
    START = 10;
    END = 2 - START;
    SENS2 = 0;
    TAO = 0.2;
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
    elseif num == 10
        x = DAMP_FACTOR;
    elseif num == 11
        x = START;
    elseif num == 12
        x = END;
    elseif num == 13
        x = SENS2;
    elseif num == 14
        x = TAO;
    end
end