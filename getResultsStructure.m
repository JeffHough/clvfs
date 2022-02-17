function [resultsStructure] = getResultsStructure(A_PRIME, A_MAX, INITIALCONDITIONSET, W_CLVF_CELL, W_LVF_CELL)

% INPUTS:
% The different sets you are testing over. I.e.,:
% a_prime - the different "safe distances" for the CLVF.
% a_max - the different max accelerations of the chaser vehicle.
% initialConditionSet - the different initial conditions you are testing.
% W_CLVF - the different CLVF weight combinations.
% W_LVF - the different LVF weight combinations.

resultsStructure.T_EST_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
resultsStructure.F_EST_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
resultsStructure.T_ACT_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
resultsStructure.F_ACT_VEC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));

resultsStructure.T_EST_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
resultsStructure.F_EST_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
resultsStructure.T_ACT_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));
resultsStructure.F_ACT_VEC_LVF = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_LVF_CELL));

resultsStructure.B = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
resultsStructure.KC = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
resultsStructure.KA = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));
resultsStructure.VMAX = zeros(numel(A_PRIME),numel(A_MAX),numel(INITIALCONDITIONSET),numel(W_CLVF_CELL));

resultsStructure.totalNumber = numel(A_MAX)*numel(INITIALCONDITIONSET)*numel(W_CLVF_CELL)*numel(A_PRIME);
end

