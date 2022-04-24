%% THIS SCRIPT IS MEANT FOR RUNNING A CLVF/LVF DOCKING APPROACH, AND COMPARING TO MPC.
clear
d2r = pi/180;
r2d = 1/d2r;

RE = 6378;
MU = 3.986 * 10^5;
J2 = -1.08264*10^-3;

%% BASIC PARAMETERS:
% SOME BASIC SIMULATION PARAMETERS:

% Length of the simulation (if required.)
T = 50000; 

% Timestep of the simulation.
timeStep = 0.05; 

%% PARAMETERS FOR SIMULATION OUTPUTS:
animateScenario = 0;
doTimeLapse = 0;

% Do the zoomed in image for the accNorm
doZoomedIn = 0;

% Show the spacecraft path in animation.
showPath = 1; 
saveImages = 0;
saveDataForAnalysis = 1;

% Which plots to produce?
plotAcc = 0;
plotThetaR = 0;
plotSpd = 0;
plotSpdErr = 0;
plotPath = 0;
plotDelV = 0;

% This was the last thing programmed into my symbolic sheet... is it it
% lol?
analysisSaveName = 'new_fuel_estimate';

%% DESCRIBE THE CHASER AND TARGET SPACECRAFT:
% Only put this code into a function for smaller setup script +
% namespacing.
SpacecraftStructure = getSpacecraftStructure(); 

% Perhaps need to add in one other inequality constraint... don't wanna
% though :(
v_max_MPC = 1;

%% SAFE DISTANCE OPTIONS:
A_PRIME = [1, 5, 10, 15, 20];  
% A_PRIME = 10;

%% MAX ACCELERATION OPTIONS:
A_MAX = [1.0, 2.0, 3.0, 4.0, 5.0]; % The maximum acceleration of your vehicle.
% A_MAX = [5];

%% INITIAL CONDITIONS SETS:    
% Options are from 0->6, with increasing intensity as we go.
INITIALCONDITIONSET = [1, 3, 5, 7];
% INITIALCONDITIONSET = [2];

%%  WEIGHT SETS: 
% options are 1->7, where 7 penalizes fuel usage the MOST.
selectedWeightIndices = [1, 3, 5, 7];
% selectedWeightIndices = 5;

% Return back the weights for the CLVF, LVF:
WeightStructure = getWeightStructure(selectedWeightIndices);

%% INITIAL CONDITIONS STRUCTURE
ICStructure = getICStructure(SpacecraftStructure, MU);

%% RESULTS STRUCTURE:
ResultsStructure = getResultsStructure(...
    A_PRIME, ...
    A_MAX, ...
    INITIALCONDITIONSET, ...
    WeightStructure.W_CLVF_CELL, ...
    WeightStructure.W_LVF_CELL...
);

%% SETUP THE SWITCHING CONDITIONS:
SwitchStructure = getSwichStructure(timeStep);

%% RUN THE SIMULATION:
% Store which run we are on:
runCnt = 0;

%% Get the control structure:
ControlStructure = getControlStructure();

%% SETUP MY NICE FIGURES:
figure
plot(0,0)
pdfprep();
close(gcf);
   
for a_prime = A_PRIME
    for a_max = A_MAX
        for initialConditionSet = INITIALCONDITIONSET
        %% RUN OBSERVATION PHASE:
        % N1 : dockingNorm.
        % N2 : rotNorm.
        % N3 : w_max.

        observationTime = 1000; % Long time(?)
        allNorms = sim("tumblingOnly");

        dockingPortNorm = max(allNorms.NORMS(:,1));
        rotNorm = max(allNorms.NORMS(:,2));
        w_max = max(allNorms.NORMS(:,3));
                
            for weightNumber = 1:numel(WeightStructure.W_CLVF_CELL)
                % Set the save name for this run:
                thisCaseSaveName = getCaseSaveName(a_max, initialConditionSet, weightNumber, a_prime);
                
                %% Pick the LVF and CLVF Weights:
                W_LVF = WeightStructure.W_LVF_CELL{weightNumber};
                W_CLVF = WeightStructure.W_CLVF_CELL{weightNumber};
                
                % Pick arbitrarily - since usually I don't want to close
                % figures for only a small number of runs.
                if ResultsStructure.totalNumber > 10
                    close all;
                end
                
                % Display progress:
                runCnt = runCnt+1;
                disp("On " + num2str(runCnt) + " of " + num2str(ResultsStructure.totalNumber));

                RC_I0 = ICStructure.rC_I0{initialConditionSet} + ICStructure.RT_I0;
                VC_I0 = ICStructure.vC_I0{initialConditionSet} + ICStructure.VT_I0;

                % Convert into meters...
                rC_T0 = ICStructure.rC_I0{initialConditionSet}*10^3;
                vC_T0 = ICStructure.vC_I0{initialConditionSet}*10^3;

                %% LVF DESIGN PROCEDURE
                % BELOW IS HOW THE VARIABLES ARE ENCODED INTO THE MAX-ACCEL FUNCTION:
                %     v_max = params(1);
                %     a_prime = params(2);
                %     w_max = rotStuff(1);
                %     theta_d = rotStuff(2);
                %     d = [rotStuff(3);rotStuff(4);rotStuff(5)];

                % INITIALIZING LYAPUNOV GUESSES:
                % An initial guess:
                v_max = 0.01; 
                fact = 95/100;
                finalAngle = fact*pi;

                muLimit = 1000;
                muFact = 10;

                tol = 10^-6;
                mu = 0.1; % My weighting parameter for the perf vs. constraints.
                gamma = 0.2; % My damping factor
                beta = 0.1; % My reduction factor.

                try
                    v_max = interiorPointLVF_heur(...
                        a_prime, ...
                        v_max, ...
                        a_max, ...
                        w_max, ...
                        rotNorm, ...
                        dockingPortNorm, ...
                        SpacecraftStructure.theta_d, ...
                        fact, ...
                        W_LVF, ...
                        mu, ...
                        tol, ...
                        gamma, ...
                        beta, ...
                        muFact, ...
                        muLimit...
                    );
                catch
                   v_max = -1; 
                end

                % What were our estimates???
                T_est_LVF = T_heur_LVF(a_prime,finalAngle,v_max);
                F_est_LVF = F_heur_LVF(a_prime,dockingPortNorm,finalAngle,v_max,rotNorm,w_max);

                %disp("LVF time estimate:")
                %disp(T_est_LVF);
                %disp("LVF deltaV estimate:")
                %disp(F_est_LVF);
                
                %% CLVF DESIGN PROCEDURE
                % PERFORMING THE DESIGN PROCEDURE OF THE THREE GAINS TO SELECT:

                aTimesOVec = SpacecraftStructure.d + a_prime*SpacecraftStructure.o_hat_prime;
                a = sqrt(sum(aTimesOVec.^2));
                o_hat_B = aTimesOVec./a;

                % INITIALIZE OUR PARAMETERS FOR THE SEARCH:
                b = 15.0;
                ka = 0.01;
                kc = 0.01;
                
                tol = 10^-5;
                mu = 0.1; % My weighting parameter for the perf vs. constraints.
                gamma = 0.2; % My damping factor
                beta = 0.1; % My reduction factor.

                muLimit = 1000;
                muFact = 10.0;
                try
                    [b, kc, ka] = interiorPointCLVF_heur(...
                        b, ...
                        kc, ...
                        ka, ...
                        a, ...
                        a_max,... 
                        w_max, ...
                        rotNorm, ...
                        rC_T0, ...
                        vC_T0, ...
                        W_CLVF, ...
                        mu, ...
                        tol, ...
                        gamma, ...
                        beta, ...
                        muFact, ...
                        muLimit...
                    );
                catch
                    b = -1;
                    kc = -1;
                    ka = -1;
                end

                % What were our estimates???
                T_est = T_heur(a,b,ka,kc,rC_T0);
                F_est = F_heur(a,b,ka,kc,rC_T0,vC_T0,rotNorm,w_max);

                %disp("T estimate is:")
                %disp(T_est);
                %disp("Fuel estimate is:")
                %disp(F_est);
                
                %% DESIGN THE MPC:
                %Q = eye(6) * W_CLVF(1);
                Q = eye(6) * 10000;
                Q_final = 100 * Q;
                R = eye(3);
                %R = eye(3) * W_CLVF(2);
                
                MPCStructure = getMPCStructure(...
                    ICStructure, ...
                    SpacecraftStructure,... 
                    R, ...
                    Q, ...
                    Q_final,... 
                    a_max * SpacecraftStructure.m,... 
                    v_max_MPC,...
                    MU, ...
                    aTimesOVec...
                );

                %% RUN THE SIMULATION
                % See if the simulation will run now:
                if b~= -1 && v_max~= -1
                    set_param('tumblingExample','StopTime',num2str(T),'FixedStep',num2str(timeStep));
                    %tic;
                    simOut = sim("tumblingExample");
                    %disp("Simulation time in seconds: " + num2str(toc));             
                    %% SAVE DATA FROM THIS RUN:
                    % Find the index to split up this vector at!
                    try
                        splitIndex = find(simOut.whichFieldToAnimate == 1,1);
                    catch
                        splitIndex = -1;
                    end

                    % Display the actual time and fuel use:
                    %disp("Actual time:")
                    %disp(simOut.t(splitIndex));

                    %disp("Actual fuel:")
                    %disp(simOut.F_act(splitIndex));

                    %disp("Actual LVF time:")
                    %disp(simOut.t(end) - simOut.t(splitIndex));

                    %disp("Actual LVF fuel:")
                    %disp(simOut.F_act(end) - simOut.F_act(splitIndex));

                    %% SAVE THE ESTIMATES AND ACTUAL TIME AND FUEL USAGE!
                    if splitIndex ~= -1 % Just skip this. Will leave default zeros.
                        alpha_index = find(A_PRIME == a_prime,1);
                        a_index = find(A_MAX == a_max,1);
                        ic_index = find(INITIALCONDITIONSET == initialConditionSet,1);
                        w_index = weightNumber;

                        ResultsStructure.T_EST_VEC(alpha_index, a_index, ic_index, w_index) = T_est;
                        ResultsStructure.F_EST_VEC(alpha_index, a_index, ic_index, w_index) = F_est;
                        ResultsStructure.T_ACT_VEC(alpha_index, a_index, ic_index, w_index) = simOut.t(splitIndex);
                        ResultsStructure.F_ACT_VEC(alpha_index, a_index, ic_index, w_index) = simOut.F_act(splitIndex);

                        ResultsStructure.T_EST_VEC_LVF(alpha_index, a_index, ic_index, w_index) = T_est_LVF;
                        ResultsStructure.F_EST_VEC_LVF(alpha_index, a_index, ic_index, w_index) = F_est_LVF;
                        ResultsStructure.T_ACT_VEC_LVF(alpha_index, a_index, ic_index, w_index) = simOut.t(end) - simOut.t(splitIndex);
                        ResultsStructure.F_ACT_VEC_LVF(alpha_index, a_index, ic_index, w_index) = simOut.F_act(end) - simOut.F_act(splitIndex);

                        %% SAVE THE SELECTED PARAMETERS:
                        ResultsStructure.B(alpha_index, a_index, ic_index, w_index) = b;
                        ResultsStructure.KC(alpha_index, a_index, ic_index, w_index) = kc;
                        ResultsStructure.KA(alpha_index, a_index, ic_index, w_index) = ka;
                        ResultsStructure.VMAX(alpha_index, a_index, ic_index, w_index) = v_max;

                        %% DO THE PLOTTING:
                        myColor = [0.9500, 0.1, 0.1];

                        % Plotting the acceleration magnitude over time:
                        if plotAcc
                            figure
                            plot(simOut.t, simOut.a_norm,'color',myColor);
                            hold on
                            grid on
                            plot([0, simOut.t(end)], [a_max, a_max],'k--');
                            lgd = legend("Acceleration magnitude", "Acceleration limit");
                            xlabel("Time (s)");
                            ylabel("Acc. magnitude (m/s^2)");
                            ylim([0 a_max*1.1])

                            % CREATE THE NEW AXES AND ZOOM-IN TO THE AREA OF INTEREST!

                            % create a new pair of axes inside current figure
                            if doZoomedIn
                                axes('position',[.3 .475 .25 .25]) 
                                box on % put box around new pair of axes
                                hold on
                                grid on
                                indexOfInterest = ... range of t near perturbation
                                    (simOut.t < higherInterestingTValue) & (simOut.t > lowerInterestingTValue); 
                                plot(simOut.t(indexOfInterest),simOut.a_norm(indexOfInterest)) % plot on new axes
                                plot([lowerInterestingTValue higherInterestingTValue],[a_max a_max],'k--')
                            end

                            if saveImages == 1

                                pause(0.01)
                                pdfplot2(gcf, thisCaseSaveName + "acc");
                            end
                        end

                        % Plotting the distance and angle theta over time:
                        if plotThetaR
                            figure
                            subplot(2,1,1);
                            grid on
                            hold on
                            plot(simOut.t(1:splitIndex-1), simOut.r(1:splitIndex-1),'color',myColor);
                            ylabel("r (m)")
                            subplot(2,1,2);
                            grid on
                            hold on
                            plot(simOut.t(1:splitIndex-1), simOut.theta(1:splitIndex-1),'color',myColor);
                            ylabel("\theta (rad)")
                            xlabel("Time (s)")

                            if saveImages == 1
                                pause(0.01)
                                pdfplot2(gcf, thisCaseSaveName + "rAndThetaCLVF");
                            end

                        % Plotting the distance and angle theta over time:
                            figure
                            subplot(2,1,1);
                            grid on
                            hold on
                            plot(simOut.t(splitIndex:end), simOut.r(splitIndex:end),'color',myColor);
                            ylabel("r_d (m)")
                            subplot(2,1,2);
                            grid on
                            hold on
                            plot([simOut.t(splitIndex), simOut.t(end)], [theta_d, theta_d],'k--','DisplayName','\theta_d');
                            plot(simOut.t(splitIndex:end), simOut.theta(splitIndex:end),"DisplayName","\theta^'",'color',myColor);
                            legend();
                            ylabel("\theta^' (rad)")
                            xlabel("Time (s)")

                            if saveImages == 1
                                pause(0.01)
                                pdfplot2(gcf, thisCaseSaveName + "rAndThetaLVF");
                            end
                        end

                        % Plotting the desired and actual speed over time:
                        if plotSpd
                            figure;
                            subplot(3,1,1);
                            grid on
                            hold on
                            plot(simOut.t, simOut.h(:,1),"k--","DisplayName","Desired");
                            plot(simOut.t, simOut.vC_I(:,1),"DisplayName","Actual",'color',myColor);
                            ylabel("$\dot{r}_x$ (m/s)","interpreter","latex")
                            legend('location','west');
                            subplot(3,1,2);
                            grid on
                            hold on
                            plot(simOut.t, simOut.h(:,2),"k--");
                            plot(simOut.t, simOut.vC_I(:,2),'color',myColor);

                            ylabel("$\dot{r}_y$ (m/s)","interpreter","latex")
                            subplot(3,1,3);
                            grid on
                            hold on
                            plot(simOut.t, simOut.h(:,3),"k--");
                            plot(simOut.t, simOut.vC_I(:,3),'color',myColor);

                            ylabel("$\dot{r}_z$ (m/s)","interpreter","latex")
                            xlabel("Time (s)")
                            if saveImages == 1
                                pause(0.01)
                                pdfplot2(gcf, thisCaseSaveName + "velProfile");
                                print(gcf,'foo','-painters','-dpdf','-r700');
                            end
                        end

                        % Plotting the velocity error over time:
                        if plotSpdErr
                            figure
                            subplot(3,1,1);
                            grid on
                            hold on
                            plot(simOut.t, simOut.h(:,1)-simOut.vC_I(:,1),'color',myColor);
                            ylabel("$\dot{r}_x$ (m/s)","interpreter","latex")
                            subplot(3,1,2);
                            grid on
                            hold on
                            plot(simOut.t, simOut.h(:,2)-simOut.vC_I(:,2),'color',myColor);
                            ylabel("$\dot{r}_y$ (m/s)","interpreter","latex")
                            subplot(3,1,3);
                            grid on
                            hold on
                            plot(simOut.t, simOut.h(:,3)-simOut.vC_I(:,3),'color',myColor);
                            ylabel("$\dot{r}_z$ (m/s)","interpreter","latex")
                            xlabel("Time (s)")
                            if saveImages == 1
                                pause(0.01)
                                pdfplot2(gcf, thisCaseSaveName + "velError");
                            end
                        end

                        %% Plot the 3D path - will help see what is going on for the underestimating of fuel:
                        if plotPath
                            figure
                            plot3(simOut.rC_T(:,1),simOut.rC_T(:,2),simOut.rC_T(:,3), 'r-', 'linewidth',2);
                            hold on
                            grid on
                            xlabel('x (m)');
                            ylabel('y (m)');
                            zlabel('z (m)');
                            axis equal
                            xlim([-2*a 2*a])
                            ylim([-2*a 2*a])
                            zlim([-2*a 2*a])
                        end

                        %% PLOT THE FUEL USAGE OVER TIME:
                        if plotDelV
                            figure
                            plot(simOut.t, simOut.F_act);
                            hold on
                            grid on
                            ylabel("Fuel usage over time (m/s)")
                            xlabel("Time (s)");
                        end

                        %% Set animate or timelapse settings:
                        animationBoxSize = 30;
                        View = [45,45];

                        % Find the first index where we are within 10 cm:
                        minInd = find(simOut.r_prime<=animationBoxSize, 1);
                        maxInd = find(simOut.r_prime<=0.1,1);

                        interestingTimeIndices = floor(linspace(minInd, maxInd,8));
                        saveName = thisCaseSaveName + "t";
                        targetColour = [0.8500, 0.3250, 0.0980];
                        chaserColour = [0.75 0.75 0.75];

                        %% Do the timelapse:
                        if doTimeLapse == 1
                            lyapunovTimeLapse(interestingTimeIndices, simOut.o_prime_T, simOut.d_T, simOut.CT_BI, simOut.rC_T, o_prime, d, animationBoxSize, View, [], [], sizT, sizC, saveImages, saveName...
                                ,showPath, targetColour, chaserColour, simOut.t);
                        end

                        %%
                        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % %%%%%%%%%%%%%%%%%%%%% RUNNING THE ANIMATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        drawPlanes = 0;
                        if animateScenario == 1
                        % lyapunovDrawAnimation(t, OT_t, psi_t, C_BIt, rT_c, OB_t, psi, animationBoxSize, View, numFrames, psi_t_future, OT_t_future)
                            lyapunovDrawAnimation(...
                                simOut.t, ...
                                simOut.o_hat_prime_T,... 
                                simOut.d_T, ...
                                simOut.CT_BI, ...
                                simOut.rC_T, ...
                                SpacecraftStructure.o_hat_prime, ...
                                SpacecraftStructure.d, ...
                                animationBoxSize, ...
                                View, ...
                                300, ...
                                [], ...
                                [], ...
                                SpacecraftStructure.sizT, ...
                                SpacecraftStructure.sizC,...
                                SpacecraftStructure,...
                                drawPlanes... a one or a zero.
                            );
                        end
                    end

                end

            end
        end
    end
end

%% SAVE THE HEURISTICS:
if saveDataForAnalysis == 1
    save(analysisSaveName,'ResultsStructure');
end
