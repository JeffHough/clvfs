% This script will be for data analysis of my .mat files...

clear
%close all
clc

%% CONTROLS:

% Do I want to make separate plots to group certain things together?
makeIndependentPlots = 0;


%% LOAD IN DATA:
% DATA = load("Time_And_Fuel_Heur_Thesis");
% DATA = load('softConstraintHeuristics');
% DATA = load('ONE_ALPHA_FIXED');
% DATA = load('alphaAndBAdjusted');
% DATA = load('alphaAndBAdjusted_NO_JERK');
% DATA=load('AandBAdj2');
% DATA = load('Non_Ad_Hoc3.mat');
% DATA = load('Non_Ad_Hoc4.mat');
% DATA = load('alteringAlphaPrime');
% DATA = load('checking_fuel_estimate_options.mat');
% DATA = load('inMathCostGradient.mat'); [CHAMP]
% DATA = load('inMathCostGradient_no_max_alpha.mat'); % DID NOT WORK WELL!!
% DATA = load('inMathCostGradient_time_reduced.mat'); % WORKS WELL,
% POSSIBLE NEW [CHAMP].
DATA = load('new_fuel_estimate.mat'); % DID NOT WORK WELL... TOO BAD :(
% :O
        
% Need to go one layer deeper:
try
    DATA = DATA.ResultsStructure;
catch
    %DATA=DATA;
end

alpha_ind = 1:size(DATA.B,1);
a_ind = 1:size(DATA.B,2);
c_ind = 1:size(DATA.B,3);
w_ind = 1:size(DATA.B,4);

if ~isfield(DATA, 'totalNumber')
   DATA.totalNumber =  numel(alpha_ind)*numel(a_ind)*numel(c_ind)*numel(w_ind);
end
   


% Do you want to save images? [not sure why this is needed...]
saveImages = 0;
    
%% PLOT HEURISTICS FOR CASES:



myColors = {...
    [0, 0.4470, 0.7410], ...	
    [0.8500, 0.3250, 0.0980],  	...
    [0.9290, 0.6940, 0.1250], ...
    [0.4940, 0.1840, 0.5560], ...
    [0.4660, 0.6740, 0.1880],...
    [0.3010, 0.7450, 0.9330], 	...
    [0.6350, 0.0780, 0.1840]...
};

myMarkers = {...
    'o', ...
    'v', ...
    '*', ...
    '^', ...
    'x', ...
    's', ...
    'd'...
};

%------------------------
% RESHAPE INTO VECTORS:
%------------------------

% CLVF:
T_EST_VEC = reshape(DATA.T_EST_VEC(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);
F_EST_VEC = reshape(DATA.F_EST_VEC(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);
T_ACT_VEC = reshape(DATA.T_ACT_VEC(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);
F_ACT_VEC = reshape(DATA.F_ACT_VEC(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);

% LVF:
T_EST_VEC_LVF = reshape(DATA.T_EST_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);
F_EST_VEC_LVF = reshape(DATA.F_EST_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);
T_ACT_VEC_LVF = reshape(DATA.T_ACT_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);
F_ACT_VEC_LVF = reshape(DATA.F_ACT_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [DATA.totalNumber,1]);

           
%% Heuristic versus actual time:                
figure()
plot(T_EST_VEC,T_ACT_VEC,'kx',"DisplayName","Simulation Data");
grid on
hold on
ylabel("Actual time, $T_{P1}$ (s)",'interpreter','latex')
xlabel("Estimated time, $\hat{T}_{P1}$ (s)",'interpreter','latex')
        
%% Heuristic versus actual fuel:    
figure()
plot(F_EST_VEC,F_ACT_VEC,'kx',"DisplayName","Simulation Data");
grid on
hold on
ylabel("Actual Fuel, $\Delta\nu_{P1}$ (m/s)",'interpreter','latex');
xlabel("Estimated Fuel, $\Delta\hat{\nu}_{P1}$ (m/s)",'interpreter','latex');   

%% LVF HEURISTIC TIME:  
figure()
plot(T_EST_VEC_LVF,T_ACT_VEC_LVF,'kx',"DisplayName","Simulation Data");
grid on
hold on
ylabel("Actual time, $T_{P2}$ (s)",'interpreter','latex')
xlabel("Estimated time, $\hat{T}_{P2}$ (s)",'interpreter','latex')

        
%% Heuristic versus actual fuel:    
figure()
plot(F_EST_VEC_LVF,F_ACT_VEC_LVF,'kx',"DisplayName","Simulation Data");
grid on
hold on
ylabel("Actual Fuel, $\Delta\nu_{P2}$ (m/s)",'interpreter','latex');
xlabel("Estimated Fuel, $\Delta\hat{\nu}_{P2}$ (m/s)",'interpreter','latex');   


%% PLOTTING BY GROUP!
    
figureNumbers = [44, 45, 46, 47];

if makeIndependentPlots
    for k = alpha_ind

    %     figureNumbers = (5*k:5*k+3);

        for i = a_ind
            for j = c_ind

                % RESHAPE THE SEGMENTS WE WANT TO PLOT:
                T_EST_VEC = reshape(DATA.T_EST_VEC(k,i,j,w_ind),[numel(DATA.T_EST_VEC(k,i,j,w_ind)),1]);
                T_ACT_VEC = reshape(DATA.T_ACT_VEC(k,i,j,w_ind),[numel(DATA.T_ACT_VEC(k,i,j,w_ind)),1]);

                F_EST_VEC = reshape(DATA.F_EST_VEC(k,i,j,w_ind),[numel(DATA.F_EST_VEC(k,i,j,w_ind)),1]);
                F_ACT_VEC = reshape(DATA.F_ACT_VEC(k,i,j,w_ind),[numel(DATA.F_ACT_VEC(k,i,j,w_ind)),1]);

                T_EST_VEC_LVF = reshape(DATA.T_EST_VEC_LVF(k,i,j,w_ind),[numel(DATA.T_EST_VEC_LVF(k,i,j,w_ind)),1]);
                T_ACT_VEC_LVF = reshape(DATA.T_ACT_VEC_LVF(k,i,j,w_ind),[numel(DATA.T_ACT_VEC_LVF(k,i,j,w_ind)),1]);

                F_EST_VEC_LVF = reshape(DATA.F_EST_VEC_LVF(k,i,j,w_ind),[numel(DATA.F_EST_VEC_LVF(k,i,j,w_ind)),1]);
                F_ACT_VEC_LVF = reshape(DATA.F_ACT_VEC_LVF(k,i,j,w_ind),[numel(DATA.F_ACT_VEC_LVF(k,i,j,w_ind)),1]);

                % Heuristic versus actual time:                
                figure(figureNumbers(1))
                hold on
                if i == 1
                    plot(T_EST_VEC,T_ACT_VEC,'color',myColors{j},'marker',myMarkers{j}, "DisplayName","Sim Case " + num2str(j));
                else
                    plot(T_EST_VEC,T_ACT_VEC,'color',myColors{j},'marker',myMarkers{j}, "HandleVisibility", "off");
                end
                grid on
                xlabel("Estimated time, $\hat{T}_{P1}$ (s)",'interpreter','latex');
                ylabel("Actual time, $T_{P1}$ (s)",'interpreter','latex');
                if i == a_ind(end)
                    legend('location','eastoutside');
                end    

                % Heuristic versus actual fuel:    
                figure(figureNumbers(2))
                if i == 1
                    plot(F_EST_VEC,F_ACT_VEC,'color',myColors{j},'marker',myMarkers{j}, "DisplayName","Sim Case " + num2str(j));
                else
                    plot(F_EST_VEC,F_ACT_VEC,'color',myColors{j},'marker',myMarkers{j}, "HandleVisibility", "off");
                end
                grid on
                hold on
                xlabel("Estimated Fuel, $\Delta\hat{\nu}_{P1}$ (m/s)",'interpreter','latex');
                ylabel("Actual Fuel, $\Delta\nu_{P1}$ (m/s)",'interpreter','latex'); 
                if i == a_ind(end)
                    legend('location','eastoutside');
                end     

                % LVF HEURISTIC TIME:  
                figure(figureNumbers(3))
                if i == 1
                    plot(T_EST_VEC_LVF,T_ACT_VEC_LVF,'color',myColors{j},'marker',myMarkers{j}, "DisplayName","Sim Case " + num2str(j));
                else
                    plot(T_EST_VEC_LVF,T_ACT_VEC_LVF,'color',myColors{j},'marker',myMarkers{j}, "HandleVisibility", "off");
                end
                grid on
                hold on
                ylabel("Actual time, $T_{P2}$ (s)",'interpreter','latex')
                xlabel("Estimated time, $\hat{T}_{P2}$ (s)",'interpreter','latex')
                if i == a_ind(end)
                    legend('location','eastoutside');
                end

                % Heuristic versus actual fuel:    
                figure(figureNumbers(4))
                if i == 1
                    plot(F_EST_VEC_LVF,F_ACT_VEC_LVF,'color',myColors{j},'marker',myMarkers{j}, "DisplayName","Sim Case " + num2str(j));
                else
                    plot(F_EST_VEC_LVF,F_ACT_VEC_LVF,'color',myColors{j},'marker',myMarkers{j}, "HandleVisibility", "off");
                end
                grid on
                hold on
                ylabel("Actual Fuel, $\Delta\nu_{P2}$ (m/s)",'interpreter','latex');
                xlabel("Estimated Fuel, $\Delta\hat{\nu}_{P2}$ (m/s)",'interpreter','latex');
                if i == a_ind(end)
                    legend('location','eastoutside');
                end

            end
        end
    end
end

