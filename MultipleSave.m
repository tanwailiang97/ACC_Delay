load VehicleLeadPrec.mat VehLead VehPrec
VehicleA = VehLead;
VehicleB = VehPrec;

param = param1;

parfor x = 1:10
    resultSave(param(x,:),0,0,VehicleA,VehicleB,x*0.1);
end