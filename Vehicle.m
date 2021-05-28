classdef Vehicle < handle
    properties
        pos = 0;                %Position
        vel = 0;                %Velocity
        acc = 0;                %Accleration
        jerk = 0;
        sig = 0;                %Desire Accleration
        tao = 0.5;              %Actuator Delay
        maxAcc = 3;             %Maximum Accleration
        minAcc = -6;            %Minimum Accleration
        period = globalVar(1);  %Global Sampling Period
        m;                      %Vehicle Mass
        a;                      %Vehicle Front Area
        %sample = 0;             %Current n
        offVel = 5;
        offPos;
    end
    
    methods
        function obj = Vehicle(mass,area,tao,maxAcc,minAcc,offVel,offPos)
            obj.m = mass;
            obj.a = area;
            obj.tao = tao;
            obj.maxAcc = maxAcc;
            obj.minAcc = minAcc;
            obj.offVel = offVel;
            obj.offPos = offPos;
        end
        
        function [pos,vel,acc] = move(obj,signal,time)
            %obj.sample = obj.sample + 1;
            
            %{
            if(signal > obj.maxAcc)
                signal = obj.maxAcc;
            elseif (signal < obj.minAcc)
                signal = obj.minAcc;
            end
            %}
            obj.sig(time) = signal;
            delay = ceil(obj.tao/obj.period);
            if time > delay
                obj.acc(time) = obj.sig(time - delay);
            else
                obj.acc(time) = 0;
            end
            
                %obj.acc(obj.sample) = signal;
                obj.vel = cumtrapz(obj.acc)*obj.period + obj.offVel;
                obj.pos = cumtrapz(obj.vel)*obj.period + obj.offPos ;
                acc = obj.acc(time);
                vel = obj.vel(time);
                pos = obj.pos(time);
           

        end
        
    end
    
end