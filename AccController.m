classdef AccController < handle

    properties
        vehLDis;    %Distance from Leading Vehicle
        vehLVel;    %Velocity different from Leading Vehicle
        vehPDis;    %Distance from Preceding Vehicle
        vehPVel;    %Velocity different from Preceding Vehicle
        sample = 1; 
        vehicle;
        k0;
        kpP;
        kvP;
        kpL;
        kvL;
        kv;
    end

    methods
        function obj = AccController(vehicle,k0,kv,kpP,kvP,kpL,kvL)
            % Inputs:
            %   vehicle :Controlled Vehicle (object)
            %   kp1 & 2 :Spring constant    (Scalar)
            %   kv1 & 2 :Damper Constant    (Scalar)
            obj.vehicle = vehicle;
            obj.k0 = k0;
            obj.kpP = kpP;
            obj.kvP = kvP;
            obj.kpL = kpL;
            obj.kvL = kvL;
            obj.kv = kv;
        end
        
        function acc = getAcc(obj,vehLDis,vehPDis,vehLVel,vehPVel,time)
            % Adaptive Cruise Control Controller  
            % Inputs:
            %   vehLDis :Distance from Leading Vehicle   (Scalar)
            %   vehPDis :Distance from Precedding Vehicle(Scalar)
            % Outputs:
            %   acc     : desire acceleration            (Scalar)

                acc = obj.kpP * vehPDis + ...
                      obj.kvP * vehPVel+...
                      obj.kpL * vehLDis + ...
                      obj.kvL * vehLVel+...
                      obj.kv  * obj.vehicle.vel(time) + ...
                      obj.k0;
        end
    end
     

end