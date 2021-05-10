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
        
        function acc = getAcc(obj,vehLDis,vehPDis,vehLVel,vehPVel)
            % Adaptive Cruise Control Controller  
            % Inputs:
            %   vehLDis :Distance from Leading Vehicle   (Scalar)
            %   vehPDis :Distance from Precedding Vehicle(Scalar)
            % Outputs:
            %   acc     : desire acceleration            (Scalar)
            
            %h = obj.vehicle.tao * 2;
            obj.vehLDis(obj.sample) = vehLDis;
            obj.vehPDis(obj.sample) = vehPDis;
            if(obj.sample>2)
                %lVel = gradient(obj.vehLDis(end-2:end));
                %pVel = gradient(obj.vehPDis(end-2:end));
                obj.vehLVel(obj.sample) = vehLVel;%lVel(end-1);
                obj.vehPVel(obj.sample) = vehPVel;%pVel(end-1);
                acc = obj.kpP * obj.vehPDis(obj.sample) + ...
                      obj.kvP * obj.vehPVel(obj.sample)+...
                      obj.kv  * obj.vehicle.vel(obj.sample) + ...
                      obj.k0;
                %{
                acc = obj.kp1 * (obj.vehLDis(obj.sample) - ...
                        2 * obj.vehLVel(obj.sample) * h ) +...
                        obj.kv1 * obj.vehLVel(obj.sample) + ...
                        obj.kp2 * (obj.vehPDis(obj.sample) - ...
                        obj.vehLVel(obj.sample) * h ) + ...
                        obj.kv2 * obj.vehPVel(obj.sample);
                %}
            else 
                acc = 0;
            end
            
            obj.sample = obj.sample + 1;
            
        end
    end
     

end