function thisCaseSaveName = getCaseSaveName(a_max, initialConditionSet, weightNumber, a_prime)

% Get the save "BASENAME" for this case.
thisCaseSaveName = "Figures/acc" + num2str(a_max) + "ic" + num2str(initialConditionSet) + "W" + num2str(weightNumber) + "alpha" + num2str(a_prime);
thisCaseSaveName = convertStringsToChars(thisCaseSaveName);
thisCaseSaveName(thisCaseSaveName == '.')='_'; % Get rid of decimals!

end

