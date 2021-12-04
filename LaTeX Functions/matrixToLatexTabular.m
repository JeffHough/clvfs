function [] = matrixToLatexTabular(A_MAX, MATRIX)

    siz = size(MATRIX);
    for k = 1:siz(1)
        thisRowStartString = "&" + "%.1f";
        thisRowEndString = "";
        for p = 1:siz(2)
            thisRowEndString = thisRowEndString + "&" + "%.3f";
        end
        thisRowEndString = thisRowEndString + "\\" + "\\" + "\n";
       fprintf(thisRowStartString+thisRowEndString, [A_MAX(k), MATRIX(k,:)]); 
    end

end

