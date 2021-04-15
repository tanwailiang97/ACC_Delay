classdef AccController < handle

    properties
        vehLDis;    %Distance from Leading Vehicle
        vehLVel;    %Velocity different from Leading Vehicle
        vehPDis;    %Distance from Preceding Vehicle
        vehPVel;    %Velocity different from Preceding Vehicle
        sample = 1; 
        vehicle;
        kp1;
        kv1;
        kp2;
        kv2;
    end

    methods
        function obj = accController(vehicle,kp1,kv1,kp2,kv2)
            % Inputs:
            %   vehicle :Controlled Vehicle (object)
            %   kp1 & 2 :Spring constant    (Scalar)
            %   kv1 & 2 :Damper Constant    (Scalar)
            obj.vehicle = vehicle;
            obj.kp1 = kp1;
            obj.kv1 = kv1;
            obj.kp2 = kp2;
            obj.kv2 = kv2;
        end
        
        function acc = getAcc(obj,vehLDis,vehPDis)
            % Adaptive Cruise Control Controller  
            % Inputs:
            %   vehLDis :Distance from Leading Vehicle   (Scalar)
            %   vehPDis :Distance from Precedding Vehicle(Scalar)
            % Outputs:
            %   acc     : desire acceleration            (Scalar)
            
            obj.vehLDis(obj.sample) = vehLDis;
            obj.vehPDis(obj.sample) = vehPDis;
            if(obj.sample>1)
                obj.vehLVel(obj.sample) = gradient(obj.vehL.pos(end-1:end));
                obj.vehPVel(obj.sample) = gradient(obj.vehP.pos(end-1:end));
                acc = obj.kp1 * obj.vehLDis(obj.sample) + ...
                        obj.kv1 * obj.vehLVel(obj.sample) + ...
                        obj.kp2 * obj.vehPDis(obj.sample) + ...
                        obj.kv2 * obj.vehPVel(obj.sample);
            end
        end
    end
     

end