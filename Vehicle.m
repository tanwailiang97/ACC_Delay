classdef Vehicle < handle
    properties
        pos = 0;                %Position
        vel = 0;                %Velocity
        acc = 0;                %Accleration
        sig = 0;                %Desire Accleration
        tao = 0.5;              %Actuator Delay
        maxAcc = 2.87;          %Maximum Accleration
        minAcc = 6;             %Minimum Accleration
        period = globalVar(1);  %Global Sampling Period
        m;                      %Vehicle Mass
        a;                      %Vehicle Front Area
        sample = 1;             %Current n
    end
    
    methods
        function obj = Vehicle(mass,area,tao,maxAcc,minAcc)
            obj.m = mass;
            obj.a = area;
            obj.tao = tao;
            obj.maxAcc = maxAcc;
            obj.minAcc = minAcc;
        end
        
        function [pos,vel,acc] = move(obj,signal)
            
            obj.sig(obj.sample) = signal;
            if(obj.sample >= obj.tao / obj.period + 1)
                accPro = obj.sig(obj.sample - obj.tao/obj.period);
            else
                accPro = 0;
            end
            
            if(accPro > obj.maxAcc)
                accPro = obj.maxAcc;
            elseif (accPro < obj.minAcc)
                accPro = obj.minAcc;
            end
            
            fAd = 0.5 * 0.32 * obj.a * 1.225 * obj.vel(obj.sample)^2; %Air Drag force
            fRr = 0.01 * obj.m * 9.81 * cos(0);     %Rolling Resistance
            if(obj.vel(obj.sample)>0)
                aDrag = (fAd + fRr) / obj.m;
            else
                aDrag = 0;
            end
            
            obj.sample = obj.sample + 1;
            obj.acc(obj.sample) = accPro - aDrag;
            obj.vel = cumtrapz(obj.acc);
            obj.pos = cumtrapz(obj.vel);
            acc = obj.acc(obj.sample);
            vel = obj.vel(obj.sample);
            pos = obj.pos(obj.sample);
            
            %fprintf('accPro = %f\naDrag = %f\nfAd = %f\nfRr = %f\n'...
            %    ,accPro,aDrag,fAd,fRr);
            
        end
        
    end
    
end