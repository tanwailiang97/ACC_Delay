classdef DelayComp < handle
    properties
        prevVel;
        prevPos;
        delInt;
        k1;
        k2;
        vehicle;
        del;
    end
    
    methods
        function obj = DelayComp(vehicle,k1,k2)
            obj.delInt = globalVar(2)/globalVar(1);
            obj.del = globalVar(2);
            obj.k1 = k1;
            obj.k2 = k2;
            obj.vehicle = vehicle;
        end
        function [pVelDiff,pPosDiff] = get(obj,VelDiff,PosDiff,time,state)
            if state == 0
                obj.prevVel = VelDiff + obj.vehicle.vel(time-obj.delInt);
                obj.prevPos = PosDiff + obj.vehicle.pos(time-obj.delInt) + 5;
                pPosDiff = 0;
                pVelDiff = 0;
            else
                Pos = PosDiff + obj.vehicle.pos(time-obj.delInt) + 5;
                Vel = VelDiff + obj.vehicle.vel(time-obj.delInt);
                Acc = gradient([obj.prevVel Vel]);
                pPos = Pos + Vel * obj.del + 0.5 * Acc(end-1) * (obj.del ^ 2);
                pPosDiff = pPos - obj.vehicle.pos(time);
                pVelDiff = Vel + Acc(end-1) * obj.del...
                            -obj.vehicle.vel(time);
            end
        end
    end
end