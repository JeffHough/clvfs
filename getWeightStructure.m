function [weightStructure] = getWeightStructure(selectWeightIndices)

% This function simply stores the different weights for the CLVF and LVF.

        % W_CLVF = [W_t, W_f, W_j];
        % Time tends to be bigger than deltaV -- 
        
        % Shifted ALL time weights down from Thesis
        
        W_CLVF0 = [10^4, 10^3];
        W_CLVF1 = [10^3, 10^3];
        W_CLVF2 = [10^2; 10^3];
        W_CLVF3 = [10^2, 10^4];
        W_CLVF4 = [10^2; 10^5];
        W_CLVF5 = [10^1, 10^5];
        W_CLVF6 = [1,    10^5];
        
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
            
        % Return these back in a structure form:
        weightStructure.W_CLVF_CELL = W_CLVF_CELL;
        weightStructure.W_LVF_CELL = W_LVF_CELL;

end

