function [] = pdfplot2(h,filename)
%PDFPLOT2 prints off a PDF from a matlab plot...

%Code Built for outputting GRAPHS as PDFS
% font change
set(0,'DefaultAxesFontName', 'Times New Roman') 
set(0,'DefaultAxesFontSize', 20)
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,filename,'-painters','-dpdf','-r700')% first entry is file name, second is type, third is resolution % WAS 700!!
end

