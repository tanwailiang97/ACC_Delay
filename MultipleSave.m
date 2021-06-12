load VehicleLeadPrec.mat VehLead VehPrec
VehicleA = VehLead;
VehicleB = VehPrec;

parfor x = 1:10
    resultSave(param(x,:),0,0,VehicleA,VehicleB,x*0.1);
end