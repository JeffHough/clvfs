function SwitchStructure = getSwichStructure(timeStep)

    d2r = pi/180;

    % Select the acceptable angle, distance, count threshold
    % until we can switch the control strategy.
    SwitchStructure.acceptableAngle = 0.5*d2r; % 
    SwitchStructure.acceptableDistance = 0.05; % 5 cm

    SwitchStructure.timeThreshold = 5; % 5 seconds staying in the radius
    SwitchStructure.cntThreshold = SwitchStructure.timeThreshold/timeStep;

end
