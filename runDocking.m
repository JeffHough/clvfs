% THIS SCRIPT IS MEANT FOR RUNNING A UAV EXAMPLE FOR THE CASCADED LYAPUNOV
% VECTOR FIELD.

    clear
%     close all
    clc
    
    d2r = pi/180;
    r2d = 1/d2r;
    
    RE = 6378;
    MU = 3.986 * 10^5;
    J2 = -1.08264*10^-3;

%% BASIC PARAMETERS:
% SOME BASIC SIMULATION PARAMETERS:
    T = 50000; % Length of the simulation (if required.)
    timeStep = 0.05; % Timestep of the simulation.
        
        animateScenario = 0; % do == 1, don't == otherwise.
        doTimeLapse = 0;
        doZoomedIn = 0; % Do the zoomed in image for the accNorm
        showPath = 1; % Show the spacecraft path in animation.
        saveImages = 0;
        saveDataForAnalysis = 1;
        
        % Which plots to produce?
            plotAcc = 0;
            plotThetaR = 0;
            plotSpd = 0;
            plotSpdErr = 0;
            plotPath = 1;
            plotDelV = 0;
        
        analysisSaveName = 'alteringAlphaPrime';
                    

        %% SAFE DISTANCE OPTIONS:
            A_PRIME = [1, 5, 10, 20];
        
        
        %% MAX ACCELERATION OPTIONS:
%             A_MAX = [1.0, 2.0, 3.0, 4.0, 5.0]; % The maximum acceleration of your vehicle.
            
            A_MAX = [1, 3, 5];
            
        %% INITIAL CONDITIONS SETS:    
        
        % Options are from 0->6, with increasing intensity as we go.
            INITIALCONDITIONSET = [0, 2, 4, 6];
            
        %%  WEIGHT SETS: 
        
        % options are 1->7, where 7 penalizes fuel usage the MOST.
            selectWeightIndices = [1, 2, 3, 4, 5, 6, 7];
            
        %% WEIGHTS:
        
        % W_CLVF = [W_t, W_f, W_j];
        % Time tends to be bigger than deltaV -- 
        
        % Shifted ALL time weights down from Thesis
        
        W_CLVF0 = [10^4, 10^3, 10^-3];
        W_CLVF1 = [10^3, 10^3, 10^-3];
        W_CLVF2 = [10^2; 10^3; 10^-3];
        W_CLVF3 = [10^2, 10^4, 10^-3];
        W_CLVF4 = [10^2; 10^5; 10^-3];
        W_CLVF5 = [10^1, 10^5, 10^-3];
        W_CLVF6 = [1,    10^5, 10^-3];
        
        W_CLVF0 = W_CLVF0./max(W_CLVF0); % Normalize
        W_CLVF1 = W_CLVF1./max(W_CLVF1); % Normalize
        W_CLVF2 = W_CLVF2./max(W_CLVF2); % Normalize
        W_CLVF3 = W_CLVF3./max(W_CLVF3); % Normalize
        W_CLVF4 = W_CLVF4./max(W_CLVF4); % Normalize
        W_CLVF5 = W_CLVF5./max(W_CLVF5); % Normalize
        W_CLVF6 = W_CLVF6./max(W_CLVF6);
        
        W_CLVF_CELL = {W_CLVF0, W_CLVF1, W_CLVF2, W_CLVF3, W_CLVF4, W_CLVF5, W_CLVF6};
        
        W_LVF0 = [10^5, 10];
        W_LVF1 = [10^5, 10^2];
        W_LVF2 = [10^4, 10^2];
        W_LVF3 = [10^3, 10^3];
        W_LVF4 = [10^2, 10^4];
        W_LVF5 = [10^2, 10^5]; 
        W_LVF6 = [10^2, 10^6];
        
        W_LVF0 = W_LVF0./max(W_LVF0); % Normalize
        W_LVF1 = W_LVF1./max(W_LVF1); % Normalize
        W_LVF2 = W_LVF2./max(W_LVF2); % Normalize
        W_LVF3 = W_LVF3./max(W_LVF3); % Normalize
        W_LVF4 = W_LVF4./max(W_LVF4); % Normalize
        W_LVF5 = W_LVF5./max(W_LVF5); % Normalize
        W_LVF6 = W_LVF6./max(W_LVF6);
        
        W_LVF_CELL = {W_LVF0 ,W_LVF1, W_LVF2, W_LVF3, W_LVF4, W_LVF5, W_LVF6};

         
        % Choose the weights according to our selections:
            W_CLVF_CELL = W_CLVF_CELL(selectWeightIndices);
            W_LVF_CELL = W_LVF_CELL(selectWeightIndices);
        
        searchMethod = "interiorPoint"; % Options - lineSearch, newtonStep, interiorPoint, gradientDescent
        costFunction = "heur"; % The heursitic one... or the not-well thought out heuristic one?
        
        xLims = [-30,30];
        yLims = [-30,30];
            
            
% First, we'll set up a target orbit:
    semiMajorAxis = 7500;
    e = 0.01;
    i = 0 * d2r;
    RAAN = 0*d2r;
    argOfPerigee = 0*d2r;
    trueAnomaly = 0*d2r;
    
% Get the initial position and velocity from the orbital elements:
    [RT_I0, VT_I0] = posAndVelFromOEs(semiMajorAxis, e, i, RAAN, argOfPerigee, trueAnomaly, MU);
    
    T_EST_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    F_EST_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    T_ACT_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    F_ACT_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    
    T_EST_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
    F_EST_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
    T_ACT_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
    F_ACT_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
    
    B = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    KC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    KA = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    VMAX = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
    
    totalNumber = numel(A_MAX)*numel(INITIALCONDITIONSET)*numel(W_CLVF_CELL)*numel(A_PRIME);
    runCnt = 0;
   
for a_prime = A_PRIME
    for a_max = A_MAX
        for initialConditionSet = INITIALCONDITIONSET
            for weightNumber = 1:numel(W_CLVF_CELL)

                runCnt = runCnt+1;
                disp("On " + num2str(runCnt) + " of " + num2str(totalNumber));

                W_CLVF = W_CLVF_CELL{weightNumber};
                W_LVF = W_LVF_CELL{weightNumber};

                W_s = 10;

            % INITIALIZE OUR PARAMETERS FOR THE SEARCH:
                b = 15.0;
                ka = 0.01;
                kc = 0.01;

                disp("a_max: " + num2str(a_max));
                disp("Initial condition number: " + num2str(initialConditionSet));
                disp("Weight number: " + num2str(weightNumber));

                % Pick arbitrarily - since usually I don't want to close
                % figures for only a small number of runs.
                if totalNumber > 10
                    close all;
                end

                    % First, declaring all of the required initial conditions:
                            if initialConditionSet == 3 % BIG VELOCITY ERROR - UNSTABLE SPIN
                                rC_I0 = [300;-500;-300]*10^-3;
    %                             vC_I0 = [-30;50;-10]*10^-3;
                                vC_I0 = [30;-50;10]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
    %                             wT_B0 = [-1.4;4.0;1.0] * d2r; 
                                wT_B0 = [-1.4;1.0;4.0]* d2r;

                            elseif initialConditionSet == 1 % FARILY NOMINAL - STABLE SPIN - MIDDLE DISTANCE
                                rC_I0 = [900;-1500;-900]*10^-3;
                                vC_I0 = [0;0;0]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
                                wT_B0 = [-1.4;1.0;2.0] * d2r; 

                            elseif initialConditionSet == 2 % FAST MINOR AXIS SPIN
                                rC_I0 = [30;-50;-30]*10^-3;
                                vC_I0 = [0;0;0]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
    %                             wT_B0 = [6.0;1.0;-1.4] * d2r;
                                wT_B0 = [-0.2; 0.8; 3.1]* d2r;

                            elseif initialConditionSet == 4 % QUITE FAST MAJOR AXIS SPIN
                                rC_I0 = [50;-30;-30]*10^-3;
                                vC_I0 = [10;-5.7;3.1]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
    %                             wT_B0 = [1.0;-1.4;9.0] * d2r;
                                wT_B0 = [1.0;-1.4;5.0] * d2r;
                            elseif initialConditionSet == 5 % VERY FAR AWAY - MAJOR/INTERMEDIATE SPIN
                                rC_I0 = [5000;-3000;-3000]*10^-3;
                                vC_I0 = [-2;3;1.0]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
                                wT_B0 = [1.0;-3.4;7.0] * d2r;
                            elseif initialConditionSet == 6 % VERY FAR - VERY FAST
                                rC_I0 = [3000;-3000;5000]*10^-3;
                                vC_I0 = [0;0;0]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
                                wT_B0 = [1.0;-2.4;10.0] * d2r;
                            elseif initialConditionSet == 0 % Good for 5 and 0.8, okay for 3.
                                rC_I0 = [300;-300;500]*10^-3;
                                vC_I0 = [0;0;0]*10^-3;
                                % Target initial angle:
                                CT_BI0 = eye(3,3);
                                wT_B0 = [0.2;0.05;-0.45] * d2r;
                            end




                    % Get the save "BASENAME" for this case.
                        thisCaseSaveName = "Figures/a" + num2str(a_max) + "case" + num2str(initialConditionSet) + "W" + num2str(weightNumber);
                        thisCaseSaveName = convertStringsToChars(thisCaseSaveName);
                        thisCaseSaveName(thisCaseSaveName == '.')='_'; % Get rid of decimals!

                        RC_I0 = rC_I0 + RT_I0;
                        VC_I0 = vC_I0 + VT_I0;

                                    % Convert into meters...
                                    rC_T0 = rC_I0*10^3;
                                    vC_T0 = vC_I0*10^3;

                    % PARAMETERS OF THE SETUP (NOT RELATED TO CLVF OR LVF DESIGN).   
                        o_prime = [1;0;0];
                        o_hat_prime = o_prime./sqrt(sum(o_prime.^2));
                        theta_d = 30*d2r;
                        d = [6;3;0]; % Might change later if it looks dumb.
                        dNorm = sqrt(sum(d.^2));
                        I = diag([35, 40, 55]);

                    %     acceptableRadius = 0.05; % a 5cm radius.

                        acceptableAngle = 0.5*d2r; % 
                        acceptableDistance = 0.05; % 5 cm

                        timeThreshold = 5; % 5 seconds staying in the radius
                        cntThreshold = timeThreshold/timeStep;

                    % OTHER PHYSICAL CHARACTERISTICS OF THE CONE BASED ON CHOSEN SPACECRAFT
                    % SIZE:

                        sizC = 2; % Size of the chaser.
                        sizT = 2; % Size of the target.

                        highConeWidth = 2*(sizT-d(2));
                        lowConeWidth = 0.5*highConeWidth;
                        coneHeight = (d(1)-sizC-sizT);

                    % Next, declaring some dynamic parameters of the scenario:
                        m = 10;
                        kd = 5.0; % P gain for the controller.


                    %% RUN OBSERVATION PHASE:

                        % N1 : dockingNorm.
                        % N2 : rotNorm.
                        % N3 : w_max.

                        observationTime = 1000; % Long time(?)
                        allNorms = sim("tumblingOnly");

                        dockingPortNorm = max(allNorms.NORMS(:,1));
                        rotNorm = max(allNorms.NORMS(:,2));
                        w_max = max(allNorms.NORMS(:,3));


                    %% LVF DESIGN PROCEDURE

                    % BELOW IS HOW THE VARIABLES ARE ENCODED INTO THE MAX-ACCEL FUNCTION:

                    %     v_max = params(1);
                    %     a_prime = params(2);
                    %     
                    %     w_max = rotStuff(1);
                    %     theta_d = rotStuff(2);
                    %     d = [rotStuff(3);rotStuff(4);rotStuff(5)];

                        % INITIALIZING LYAPUNOV GUESSES:
                            v_max = 0.01; % The maximum speed.
    %                         a_prime = 0.01;
                            fact = 95/100;
                            finalAngle = fact*pi;


                        if searchMethod == "lineSearch"

                            a_prime = 4.9577;

                            % INITIALIZE PARAMETERS FOR THE LINE-SEARCH
                                params = [v_max, a_prime]; % The parameter line to search.

                            % SELECT OUR MAXIMUM ROTATION RATES ACCORDING TO THE TRAJECTORY:
                                rotStuff = [dockingPortNorm, rotNorm, w_max, theta_d, fact]; % The angular data for the target.

                             % PICK OUR SEARCH DIRECTION FOR THE PARAMETER LINE-SEARCH:
                                searchDirection = [0.1,0.0]; % Only allow v_max to vary.

                            % PICK A DESIRED ACCURACY, STEP SIZE, SHRINK FACTOR.
                                desiredAccuracy = 0.0001;
                                stepSize = 0.001;
                                stepShrinkingFactor = 0.5;

                            % SELECT NEW PARAMETERS:
                                newParams = lineSearch(params, 'a_max_LVF3D', searchDirection, desiredAccuracy, a_max, stepSize, stepShrinkingFactor, rotStuff);

                            % OUTPUT THE LYAPUNOV VECTOR FIELD PARAMETERS:
                                v_max = newParams(1);
                                a_prime = newParams(2);


                        elseif searchMethod == "newtonStep"

                            tol = 10^-5;
                            mu = 100; % My weighting parameter for the perf vs. constraints.
                            gamma = 1; % My damping factor
                            beta = 0.8; % My reduction factor.

                            % LEGEND OF WEIGHTS:
                    %         % For time:
                    %             W_t_vmax = W_LVF(1);
                    % 
                    %         % For fuel:
                    %             W_f_alphap = W_LVF(2);
                    %             W_f_vmax = W_LVF(3);
                    % 
                    %         % For risk:
                    %             W_r = W_LVF(4);

                            W_LVF = [1, 1, 100, 1000];
                            [a_prime, v_max] = newtonStepLVF(a_prime, v_max, a_max, w_max, rotNorm, dockingPortNorm, theta_d, fact, W_LVF, mu, tol, gamma, beta);
                            disp("a_prime is");
                            disp(a_prime)
                            disp("v_max is");
                            disp(v_max);

                        elseif searchMethod == "interiorPoint"

                            muLimit = 1000;
                            muFact = 10;

                            tol = 10^-6;
                            mu = 0.1; % My weighting parameter for the perf vs. constraints.
                            gamma = 0.2; % My damping factor
                            beta = 0.1; % My reduction factor.

                            % LEGEND OF WEIGHTS:
                             % For time:
                                % W_LVF = [ W_t_vmax, W_f_alphap, W_f_vmax,
                                % W_r];

                            if costFunction == "heur"

    %                             a_prime = 4.8; % In meters.
                                v_max = interiorPointLVF_heur(a_prime, v_max, a_max, w_max, rotNorm, dockingPortNorm, theta_d, fact, W_LVF, mu, tol, gamma, beta, muFact, muLimit);

                            else

                                W_LVF = [1, 1, 100, 1000];
                                [a_prime, v_max] = interiorPointLVF(a_prime, v_max, a_max, w_max, rotNorm, dockingPortNorm, theta_d, fact, W_LVF, mu, tol, gamma, beta, muFact, muLimit);

                            end

                        end

                        % What were our estimates???
                            T_est_LVF = T_heur_LVF(a_prime,finalAngle,v_max);
                            F_est_LVF = F_heur_LVF(a_prime,dockingPortNorm,finalAngle,v_max,rotNorm,w_max);




                    %% CLVF DESIGN PROCEDURE
                    % PERFORMING THE DESIGN PROCEDURE OF THE THREE GAINS TO SELECT:

                                aTimesOVec = d + a_prime*o_hat_prime; % All in the body-fixed frame.
                                a = sqrt(sum(aTimesOVec.^2));
                                o_hat_B = aTimesOVec./a;

                            if searchMethod == "lineSearch"

                                % INITIALIZE PARAMETERS FOR THE LINE-SEARCH
                                    params = [kc, ka, b, a]; % The parameter line to search.

                                % SELECT OUR MAXIMUM ROTATION RATES ACCORDING TO THE TRAJECTORY:
                                    rotStuff = [rotNorm,w_max]; % The angular data for the target.

                                % PICK OUR SEARCH DIRECTION FOR THE PARAMETER LINE-SEARCH:
                                    searchDirection = [0.1,0.1,-0.5,0]; % USUAL ONE!

                                % PICK A DESIRED ACCURACY, STEP SIZE, SHRINK FACTOR.
                                    desiredAccuracy = 0.0001;
                                    stepSize = 0.001;
                                    stepShrinkingFactor = 0.5;

                                % SELECT NEW PARAMETERS:
                                    newParams = lineSearch(params, 'a_max_CLVF3D', searchDirection, desiredAccuracy, a_max, stepSize, stepShrinkingFactor, rotStuff);

                                    kc = newParams(1);
                                    ka = newParams(2);
                                    b = newParams(3);

    %                                 disp("ka is")
    %                                 disp(ka)
    %                                 disp("kc is")
    %                                 disp(kc)
    %                                 disp("b is")
    %                                 disp(b)

                            elseif searchMethod == "newtonStep"
                                tol = 10^-8;
                                mu = 100; % My weighting parameter for the perf vs. constraints.
                                gamma = 1; % My damping factor
                                beta = 0.8; % My reduction factor.

                                % LEGEND OF WEIGHTS:

                    %             W_CLVF = [W_t_kc, W_t_ka, W_f_kc, W_f_ka, W_c, W_t_b];

                                W_CLVF = [100, 100, 10, 10, 1000, 1];
                                W_CLVF = W_CLVF./max(W_CLVF); % Normalize
                                [b, kc, ka] = newtonStepCLVF_quad(b, kc, ka, a, a_max, w_max, rotNorm, W_CLVF, mu, tol, gamma, beta);


                            elseif searchMethod == "interiorPoint"

                                tol = 10^-5;
                                mu = 0.1; % My weighting parameter for the perf vs. constraints.
                                gamma = 0.2; % My damping factor
                                beta = 0.1; % My reduction factor.

                                muLimit = 1000;
                                muFact = 10.0;

                                if costFunction == "heur"

                                    % W_CLVF = [W_t, W_f, W_j];

                                    [b, kc, ka] = interiorPointCLVF_heur(b, kc, ka, a, a_max, w_max, rotNorm, rC_T0 , vC_T0, W_CLVF, mu, tol, gamma, beta, muFact, muLimit);%, r_factor, W_s);


                                else

                                    % LEGEND OF WEIGHTS:
                    %               W_CLVF = [W_t_kc, W_t_ka, W_f_kc, W_f_ka, W_c, W_t_b];
                                    W_CLVF = [10^6, 10^4, 10^2, 10^2, 10^2, 10^6];
                                    W_CLVF = W_CLVF./max(W_CLVF); % Normalize

                                    [b, kc, ka] = interiorPointCLVF_quad(b, kc, ka, a, a_max, w_max, rotNorm, W_CLVF, mu, tol, gamma, beta, muFact, muLimit);


                                end

                            end

                        % What were our estimates???
                            T_est = T_heur(a,b,ka,kc,rC_T0);
                            F_est = F_heur(a,b,ka,kc,rC_T0,vC_T0,rotNorm,w_max);


                        disp("T estimate is:")
                        disp(T_est);
                        disp("Fuel estimate is:")
                        disp(F_est);

                        disp("LVF time estimate:")
                        disp(T_est_LVF);
                        disp("LVF deltaV estimate:")
                        disp(F_est_LVF);


                    %% RUN THE SIMULATION

                    % See if the simulation will run now:
    %                     disp("Running the Docking example...");
                        set_param('tumblingExample','StopTime',num2str(T),'FixedStep',num2str(timeStep));
                        simOut = sim("tumblingExample");

                    %% SETUP MY NICE FIGURES:

                        figure
                        pdfprep();
                        close(gcf);


                    %% SAVE DATA FROM THIS RUN:  

                    % Find the index to split up this vector at!
                        splitIndex = find(simOut.whichFieldToAnimate == 1,1);

                    % Display the actual time and fuel use!
                        disp("Actual time:")
                        disp(simOut.t(splitIndex));

                        disp("Actual fuel:")
                        disp(simOut.F_act(splitIndex));

                        disp("Actual LVF time:")
                        disp(simOut.t(end) - simOut.t(splitIndex));

                        disp("Actual LVF fuel:")
                        disp(simOut.F_act(end) - simOut.F_act(splitIndex));


                    %% SAVE THE ESTIMATES AND ACTUAL TIME AND FUEL USAGE!
                        alpha_index = find(A_PRIME == a_prime,1);
                        a_index = find(A_MAX == a_max,1);
                        ic_index = find(INITIALCONDITIONSET == initialConditionSet,1);
                        w_index = weightNumber;

                        T_EST_VEC(alpha_index, a_index, ic_index, w_index) = T_est;
                        F_EST_VEC(alpha_index, a_index, ic_index, w_index) = F_est;
                        T_ACT_VEC(alpha_index, a_index, ic_index, w_index) = simOut.t(splitIndex);
                        F_ACT_VEC(alpha_index, a_index, ic_index, w_index) = simOut.F_act(splitIndex);

                        T_EST_VEC_LVF(alpha_index, a_index, ic_index, w_index) = T_est_LVF;
                        F_EST_VEC_LVF(alpha_index, a_index, ic_index, w_index) = F_est_LVF;
                        T_ACT_VEC_LVF(alpha_index, a_index, ic_index, w_index) = simOut.t(end) - simOut.t(splitIndex);
                        F_ACT_VEC_LVF(alpha_index, a_index, ic_index, w_index) = simOut.F_act(end) - simOut.F_act(splitIndex);

                    %% SAVE THE SELECTED PARAMETERS:
                        B(alpha_index, a_index, ic_index, w_index) = b;
                        KC(alpha_index, a_index, ic_index, w_index) = kc;
                        KA(alpha_index, a_index, ic_index, w_index) = ka;
                        VMAX(alpha_index, a_index, ic_index, w_index) = v_max;


                    %% DO THE PLOTTING:

                    myColor = [0.9500, 0.1, 0.1];

                    figure
                    plot(0,0)
                    pdfprep;
                    close(gcf)

                    % Plotting the acceleration magnitude over time:
                    if plotAcc
                        figure
                        plot(simOut.t, simOut.a_norm,'color',myColor);
                        hold on
                        grid on
                        plot([0, simOut.t(end)], [a_max, a_max],'k--');
                        lgd = legend("Acceleration magnitude", "Acceleration limit");
    %                     lgd.Location = 'best';
                        xlabel("Time (s)");
                        ylabel("Acc. magnitude (m/s^2)");
                        ylim([0 a_max*1.1])

    %                     CREATE THE NEW AXES AND ZOOM-IN TO THE AREA OF INTEREST!

                        % create a new pair of axes inside current figure
                        if doZoomedIn
                            axes('position',[.3 .475 .25 .25]) 
                            box on % put box around new pair of axes
                            hold on
                            grid on
                            indexOfInterest = (simOut.t < higherInterestingTValue) & (simOut.t > lowerInterestingTValue); % range of t near perturbation
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
                        plot3(simOut.rC_T(:,1),simOut.rC_T(:,2),simOut.rC_T(:,3));
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
                            minInd = find(simOut.r<=animationBoxSize, 1);
                            maxInd = find(simOut.r<=0.1,1);

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
                    % 
                        if animateScenario == 1
                        % lyapunovDrawAnimation(t, OT_t, psi_t, C_BIt, rT_c, OB_t, psi, animationBoxSize, View, numFrames, psi_t_future, OT_t_future)
                            lyapunovDrawAnimation(simOut.t, simOut.o_prime_T, simOut.d_T, simOut.CT_BI, simOut.rC_T, o_prime, d, animationBoxSize, View, 300, [], [], sizT, sizC);
                        end


                    %     % SAVE THIS FIGURE:
                    %     if saveImages
                    %         pdfplot2(gcf, "FIGS/" + caseTitle + "rAndTheta" + ACC_TITLE);
                    %     end

                        % FOR THE UAV EXAMPLE TRAJECTORY:
                            % -> 150-160 IS THE TURNING.
                            % -> 160-190 IS THE DRIVING.

            end
        end
    end
end

%% SAVE THE HEURISTICS:


    if saveDataForAnalysis == 1
        save(analysisSaveName,'T_EST_VEC','T_ACT_VEC','F_EST_VEC','F_ACT_VEC','T_EST_VEC_LVF','T_ACT_VEC_LVF','F_EST_VEC_LVF','F_ACT_VEC_LVF','B','KC','KA','VMAX');
    end




