% This script will be for data analysis of my .mat files...

clear
close all
clc

%% LOAD IN DATA:
%     DATA = load("Time_And_Fuel_Heur_Thesis");
%     DATA = load('softConstraintHeuristics');
%     DATA = load('ONE_ALPHA_FIXED');
%     DATA = load('alphaAndBAdjusted');
%     DATA = load('alphaAndBAdjusted_NO_JERK');
%     DATA=load('AandBAdj2');
%     DATA = load('Non_Ad_Hoc3.mat'); % BEST SO FAR!!
%     DATA = load('Non_Ad_Hoc4.mat');
    DATA = load('alteringAlphaPrime');


    % The structure is like this:
        % T_EST_VEC = zeros(numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
        
    % Do you want to save images?
        saveImages = 0;



    
%% PLOT HEURISTICS FOR CASES:
    
    alpha_ind = 1:size(DATA.B,1);
    a_ind = 1:size(DATA.B,2);
    c_ind = 1:size(DATA.B,3);
    w_ind = 1:size(DATA.B,4);
        
    myColors = {[0, 0.4470, 0.7410], 	[0.8500, 0.3250, 0.0980],  	[0.9290, 0.6940, 0.1250], [0.4940, 0.1840, 0.5560], [0.4660, 0.6740, 0.1880],...
        [0.3010, 0.7450, 0.9330], 	[0.6350, 0.0780, 0.1840]};
    
    myMarkers = {'o', 'v', '*', '^', 'x', 's', 'd'};
    

    % CAN WE GROUP TOGETHER THE A AND C INTO COLOURS? THAT WAY, WE CAN SHOW
    % THE FIFTH INDEX (W_IND) TENDING TO GO OFF-PATTERN...
    
% First, we'll reshape:

    % RESHAPE INTO VECTORS:
        T_EST_VEC = reshape(DATA.T_EST_VEC(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.T_EST_VEC(alpha_ind, a_ind, c_ind, w_ind)),1]);
        F_EST_VEC = reshape(DATA.F_EST_VEC(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.F_EST_VEC(alpha_ind, a_ind, c_ind, w_ind)),1]);
        T_ACT_VEC = reshape(DATA.T_ACT_VEC(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.T_ACT_VEC(alpha_ind, a_ind, c_ind, w_ind)),1]);
        F_ACT_VEC = reshape(DATA.F_ACT_VEC(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.F_ACT_VEC(alpha_ind, a_ind, c_ind, w_ind)),1]);
        
        T_EST_VEC_LVF = reshape(DATA.T_EST_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.T_EST_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind)),1]);
        F_EST_VEC_LVF = reshape(DATA.F_EST_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.F_EST_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind)),1]);
        T_ACT_VEC_LVF = reshape(DATA.T_ACT_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.T_ACT_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind)),1]);
        F_ACT_VEC_LVF = reshape(DATA.F_ACT_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind), [numel(DATA.F_ACT_VEC_LVF(alpha_ind, a_ind, c_ind, w_ind)),1]);
        
        % FIT THE TEST AND TACT:
            pt = polyfit(T_ACT_VEC,T_EST_VEC,1);
            f_time = @(x) polyval(pt,x);
            
        % FIT THE FEST AND FACT:
            pf = polyfit(F_ACT_VEC, F_EST_VEC,1);
            f_fuel = @(x) polyval(pf,x);
  
        % FIT THE TEST AND TACT:
            pt_LVF = polyfit(T_ACT_VEC_LVF,T_EST_VEC_LVF,1);
            f_time_LVF = @(x) polyval(pt_LVF,x);
            
        % FIT THE FEST AND FACT:
            pf_LVF = polyfit(F_ACT_VEC_LVF, F_EST_VEC_LVF,1);
            f_fuel_LVF = @(x) polyval(pf_LVF,x);
            
            
    thisCaseSaveName = "dataFigures/A" + num2str(a_ind(1)) + num2str(a_ind(end)) + "C" + num2str(c_ind(1)) + num2str(c_ind(end)) + ...
               "W" + num2str(w_ind(1)) + num2str(w_ind(end))+"ALPHA"+num2str(alpha_ind(1)) + num2str(alpha_ind(end));


    if saveImages == 1
        figure
        plot(0,0);
        pdfprep();
        close(gcf);
    end
           
    %% Heuristic versus actual time:                
        figure(1)
        plot(T_EST_VEC,T_ACT_VEC,'kx',"DisplayName","Simulation Data");
        grid on
        hold on
        ylabel("Actual time, $T_{P1}$ (s)",'interpreter','latex')
        xlabel("Estimated time, $\hat{T}_{P1}$ (s)",'interpreter','latex')
        if saveImages == 1
           saveName = thisCaseSaveName + "ta";
           pdfplot2(gcf, saveName);
        end
%         fplot(f_time,"DisplayName","T_e = " + num2str(pt(1))+"T_a + " + num2str(pt(2)));
%         legend();
        
    %% Heuristic versus actual fuel:    
        figure(2)
        plot(F_EST_VEC,F_ACT_VEC,'kx',"DisplayName","Simulation Data");
        grid on
        hold on
        ylabel("Actual Fuel, $\Delta\nu_{P1}$ (m/s)",'interpreter','latex');
        xlabel("Estimated Fuel, $\Delta\hat{\nu}_{P1}$ (m/s)",'interpreter','latex');   
        if saveImages == 1
           saveName = thisCaseSaveName + "delva";
           pdfplot2(gcf, saveName);
        end
%         fplot(f_fuel,"DisplayName","F_e = " + num2str(pf(1))+"F_a + " + num2str(pf(2)));
%         legend();

    %% LVF HEURISTIC TIME:  
        figure(3)
        plot(T_EST_VEC_LVF,T_ACT_VEC_LVF,'kx',"DisplayName","Simulation Data");
        grid on
        hold on
        ylabel("Actual time, $T_{P2}$ (s)",'interpreter','latex')
        xlabel("Estimated time, $\hat{T}_{P2}$ (s)",'interpreter','latex')
        if saveImages == 1
           saveName = thisCaseSaveName + "tc";
           pdfplot2(gcf, saveName);
        end
%         fplot(f_time_LVF,"DisplayName","T_e = " + num2str(pt(1))+"T_a + " + num2str(pt(2)));
%         legend();
        
    %% Heuristic versus actual fuel:    
        figure(4)
        plot(F_EST_VEC_LVF,F_ACT_VEC_LVF,'kx',"DisplayName","Simulation Data");
        grid on
        hold on
        ylabel("Actual Fuel, $\Delta\nu_{P2}$ (m/s)",'interpreter','latex');
        xlabel("Estimated Fuel, $\Delta\hat{\nu}_{P2}$ (m/s)",'interpreter','latex');   
        if saveImages == 1
           saveName = thisCaseSaveName + "delvc";
           pdfplot2(gcf, saveName);
        end
%         fplot(f_fuel_LVF,"DisplayName","F_e = " + num2str(pf(1))+"F_a + " + num2str(pf(2)));
%         legend();

    %% Ratio of kc/ka

%% PLOTTING BY GROUP!
% NEED TO REDO THIS PART.... GROUP AGAIN BY DIFFERENT SAFE DISTANCES??
    
% First, we'll reshape:
for k = alpha_ind
    
    figureNumbers = (5*k:5*k+3);
    
    for i = a_ind
        for j = c_ind

            T_EST_VEC = reshape(DATA.T_EST_VEC(k,i,j,w_ind),[numel(DATA.T_EST_VEC(k,i,j,w_ind)),1]);
            T_ACT_VEC = reshape(DATA.T_ACT_VEC(k,i,j,w_ind),[numel(DATA.T_ACT_VEC(k,i,j,w_ind)),1]);

            F_EST_VEC = reshape(DATA.F_EST_VEC(k,i,j,w_ind),[numel(DATA.F_EST_VEC(k,i,j,w_ind)),1]);
            F_ACT_VEC = reshape(DATA.F_ACT_VEC(k,i,j,w_ind),[numel(DATA.F_ACT_VEC(k,i,j,w_ind)),1]);

            T_EST_VEC_LVF = reshape(DATA.T_EST_VEC_LVF(k,i,j,w_ind),[numel(DATA.T_EST_VEC_LVF(k,i,j,w_ind)),1]);
            T_ACT_VEC_LVF = reshape(DATA.T_ACT_VEC_LVF(k,i,j,w_ind),[numel(DATA.T_ACT_VEC_LVF(k,i,j,w_ind)),1]);

            F_EST_VEC_LVF = reshape(DATA.F_EST_VEC_LVF(k,i,j,w_ind),[numel(DATA.F_EST_VEC_LVF(k,i,j,w_ind)),1]);
            F_ACT_VEC_LVF = reshape(DATA.F_ACT_VEC_LVF(k,i,j,w_ind),[numel(DATA.F_ACT_VEC_LVF(k,i,j,w_ind)),1]);

    %     % Heuristic versus actual time:                
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
            
    %     Heuristic versus actual fuel:    
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
            
    %     % LVF HEURISTIC TIME:  
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

    if ishandle(5)
            if saveImages == 1
               saveName = thisCaseSaveName + "taGROUPS";
               pdfplot2(figure(5), saveName);
            end
    end

    if ishandle(6)
            if saveImages == 1
               saveName = thisCaseSaveName + "delvaGROUPS";
               pdfplot2(figure(6), saveName);
            end
    end
    
    if ishandle(7)
            if saveImages == 1
               saveName = thisCaseSaveName + "tcGROUPS";
               pdfplot2(figure(7), saveName);
            end
    end

    if ishandle(8)
            if saveImages == 1
               saveName = thisCaseSaveName + "delvcGROUPS";
               pdfplot2(figure(8), saveName);
            end
    end






