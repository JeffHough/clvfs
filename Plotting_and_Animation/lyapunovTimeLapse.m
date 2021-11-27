function [] = lyapunovTimeLapse(indices, OT_t, d_T, CT_BI, rC_T, OB_t, d, animationBoxSize, View, psi_t_future, OT_t_future, sizT, sizC, saveFigures, saveName,...
    showPath, targetColour, chaserColour, tVec)
% Used to create timelapse figures for papers on Lyapunov Vector Fields.


    figure
    
%     scColour = [0.75 0.75 0.75];

    hold on;

    for i = indices
        z_end = 100*OT_t(i,:)' + d_T(i,:)';
        z = plot3([d_T(i,1); z_end(1)], [d_T(i,2); z_end(2)], [d_T(i,3); z_end(3)],'k-', "DisplayName","Align Dir.","LineWidth",2);
        
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
            
        target = drawspacecraft(sizT,OB_t,d,0.8*sizT,CT_BI(:,:,i),[0;0;0], targetColour, 4/9*sizT, 7/9*sizT, "Target");
        chaser = drawspacecraft(sizC,[1;0;0],dNorm*[1;0;0],0.8*sizT,CC_BI,rC_T(i,:)', chaserColour, 7/9*sizT, 4/9*sizT, "Chaser");
        zlim([-animationBoxSize animationBoxSize])
        xlim([-animationBoxSize animationBoxSize])
        ylim([-animationBoxSize animationBoxSize])
        
        if showPath == 1
            thePath = plot3(rC_T(1:i,1), rC_T(1:i,2), rC_T(1:i,3),"color",[0.8500, 0.3250, 0.0980], "DisplayName","Path","LineWidth",2);
        end
        
        legend();
        
        view(View);
        grid on
        xlabel("x (m)")
        ylabel("y (m)")
        zlabel("z (m)")
        drawnow limitrate;

        
        if saveFigures == 1
            % Create the actual saveName:
                fullSaveName = convertStringsToChars(saveName + num2str(tVec(i)));
                fullSaveName(fullSaveName == '.') = '_'; % Get rid of decimals.
                
            pdfplot2(gcf,fullSaveName);
        end
        
        %         pause(0.01);
        delete(target)
        delete(chaser)
        delete(z)
        if showPath == 1
            delete(thePath);
        end
        if ~isempty(psi_t_future)
            delete(z_future)
        end
    end
