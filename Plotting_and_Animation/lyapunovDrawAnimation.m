function [] = lyapunovDrawAnimation(t, OT_t, d_T, CT_BI, rC_T, OB_t, d, animationBoxSize, View, numFrames, psi_t_future, OT_t_future, sizT, sizC)

    % First, we'll adjust the indices to not include any where we cannot
    % see the target.
    
    rNorms = zeros(numel(t),1);
    
    for k = 1:numel(rNorms)
        rNorms(k) = sqrt(sum(rC_T(k,:).^2));
    end
    
    firstInterestingTimeIndex = find(rNorms <= animationBoxSize,1);

%     figure
    gcf;
    indicies = linspace(firstInterestingTimeIndex, numel(t),numFrames);
    indicies = floor(indicies);
    
    scColour = [0.75 0.75 0.75];


    hold on;

    for i = indicies
        z_end = 100*OT_t(i,:)' + d_T(i,:)';
        z = plot3([d_T(i,1); z_end(1)], [d_T(i,2); z_end(2)], [d_T(i,3); z_end(3)],'k-');
        
        if ~isempty(psi_t_future)
            z_end_future = 100*OT_t_future(i,:)' + psi_t_future(i,:)';
            z_future = plot3([psi_t_future(i,1); z_end_future(1)], [psi_t_future(i,2); z_end_future(2)], [psi_t_future(i,3); z_end_future(3)],'r-');
        end
        
        
        % Give the chaser a different orientation from the target (page 257
        % in nonlinear systems/latest research book).
        % Position of TARG rel to CHAS:
            rD_C = -(rC_T(i,:)' - (d_T(i,:) - sizT*0.05*OT_t(i,:))');
            firstAngle = -atan2(-rD_C(2), rD_C(1));
            rNorm = sqrt(sum(rD_C.^2));
            secondAngle = -asin(rD_C(3)/rNorm);
            
        % Get the z-axis of the 
            ourRotationSoFar = C2(secondAngle)*C3(firstAngle);
            ourY = ourRotationSoFar(2,:)';
            ourZ = ourRotationSoFar(3,:)';
            
        % Get the target z-axis in inertial coordinates:
            targZ = CT_BI(3,:,i)';
            
        % Get the targZ y and z coordinates in our frame:
            targZ_y = targZ'*ourY;
            targZ_z = targZ'*ourZ;
            
        % Get the final angle:
            thirdAngle = atan2(targZ_y, -targZ_z);
            
        % Our final rotation matrix:
            CC_BI = C1(thirdAngle)*ourRotationSoFar;
        
        dNorm = sqrt(sum(d.^2));
            
        target = drawspacecraft(sizT,OB_t,d,0.8*sizT,CT_BI(:,:,i),[0;0;0], scColour, 4/9*sizT, 7/9*sizT, "Target");
        chaser = drawspacecraft(sizC,[1;0;0],dNorm*[1;0;0],0.8*sizT,CC_BI,rC_T(i,:)', scColour, 7/9*sizT, 4/9*sizT, "Chaser");
        zlim([-animationBoxSize animationBoxSize])
        xlim([-animationBoxSize animationBoxSize])
        ylim([-animationBoxSize animationBoxSize])
        view(View);
        grid on
        drawnow limitrate;
%         pause(0.01);
        delete(target)
        delete(chaser)
        delete(z)
        if ~isempty(psi_t_future)
            delete(z_future)
        end
    end

