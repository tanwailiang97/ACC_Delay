classdef DelayComp < handle
    properties
        prevVel = 0;
        prevPos = 0;
        delInt;
        k1;
        k2;
        vehicle;
        del;
    end
    
    methods
        function obj = DelayComp(vehicle,k1,k2,del)
            obj.delInt = round(del/globalVar(1));
            obj.del = del;
            obj.k1 = k1;
            obj.k2 = k2;
            obj.vehicle = vehicle;
        end
        function [pVelDiff,pPosDiff] = get(obj,VelDiff,PosDiff,time,state,AccP)
            if state == 0
                obj.prevVel = VelDiff + obj.vehicle.vel(time-obj.delInt);
                obj.prevPos = PosDiff + obj.vehicle.pos(time-obj.delInt);
                pPosDiff = 0;
                pVelDiff = 0;
            elseif state == 1
                Pos = PosDiff + obj.vehicle.pos(time-obj.delInt);
                Vel = VelDiff + obj.vehicle.vel(time-obj.delInt);
                Acc = gradient([obj.prevVel Vel]);
                pPos = Pos + Vel * obj.del + 0.5 * Acc(1) * (obj.del ^ 2);
                pPosDiff = pPos - obj.vehicle.pos(time);
                pVelDiff = Vel + Acc(1) * obj.del...
                            -obj.vehicle.vel(time);
                        
            elseif state == 2
                Pos = PosDiff + obj.vehicle.pos(time-obj.delInt);
                Vel = VelDiff + obj.vehicle.vel(time-obj.delInt);
                Acc = gradient([obj.prevVel Vel]);
                obj.prevVel = Vel;
                pPos = Pos + Vel * obj.del + 0.5 * Acc(1) * (obj.del ^ 2);
                pPosDiff = pPos - obj.vehicle.pos(time);
                pVelDiff = Vel + Acc(1) * obj.del...
                            -obj.vehicle.vel(time);
            end
        end
    end
end