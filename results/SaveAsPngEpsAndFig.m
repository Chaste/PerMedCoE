% Save a figure as a .png .eps and .fig and adjust font sizes according to the
% expected graph size in the .pdf file
% filename          file name WITHOUT EXTENSION
% h                 list of plot handles in the current figure whose marker size or
%                   line width should be modified. If h is -1, then all
%                   markers/lines in the figure have their sizes changed
% pdfwidth          graph width in the pdf file
% widthheightratio  16/9, 4/3, Pi whatever you prefer
% pdflabelsize      desired label font size in the pdf file. Axis tick
%                   labels and legends have a fontsize of 0.8xpdflabelsize 
%
%
% Warnings: 
% 1 - the current figure should be the one we want to export (i.e. function acts on 'gcf')
% 2 - at the end of the function axis object is in manual mode. Type 'axis
% auto' to put them in 'auto mode'. Units of the gcf are also modified to
% "centimeters"
% 3 - the first 5 parameters defined just after the function declaration
% could be parameters. Sensibly modifying their value should not cause the
% function to fail. Reasonable ranges are indicated between brackets.
% 4 - generated pdf and eps do not look exactly the same. Nothing I can't
% do about that.
% 5 - Not tested for 3D plots. It might work though.
%
% Edited by Ozzy on 11/08/2011

function SaveAsPngEpsandFig(h,filename, pdfwidth, widthheightratio,pdflabelsize)


linewidth = 0.8;                              % line width in ? (between 1 and 3)
markersize = 1;                             % size of markers in pt (1-20)
labeloveraxislabelratio = 0.8;              % ratio between label font size and axis label font size (between 0 and 1)
gridwidth = 0.5;                              % line width of the grid (between 1 and 3)
resolution = 300;                           % resolution in dpi (between 150 and 600)


set(gcf, 'Units','centimeters')             % Change units of the gcf to centimeters
dimensions = [pdfwidth (pdfwidth/widthheightratio)]         % Work out new dimensions
labelsize = pdflabelsize;                       
axeslabelsize =floor(labeloveraxislabelratio*labelsize);    % Work out axis label font size

hold on;
axis manual                                  % to prevent scale change when resizing

set(gca,'fontsize',axeslabelsize);           % set font size of the axes tick labels
set(gca,'linewidth',gridwidth);              % set line width of axis and grid (not plots)
set(gcf,'position', [0,0, dimensions]);      % set display size
set(gcf,'PaperPositionMode','auto');         % Prevent redimensioning by the print function
set(get(gca,'XLabel'),'fontsize',labelsize); % set font size for xlabel
set(get(gca,'YLabel'),'fontsize',labelsize); % set font size for ylabel
set(get(gca,'ZLabel'),'fontsize',labelsize); % set font size for zlabel
set(get(gca,'Title'),'fontsize',labelsize);  % set font size for title

if (h == -1)                                 % if plot handles that need line width change is -1
    h = get(gca,'Children');                 % do it for all
end

for i=1:length(h)
    set(h(i), 'linewidth' , linewidth);      % set line width
    %set(h(i), 'MarkerSize' , markersize);    % set marker size
end

resolution_str = ['-r' num2str(resolution)];

%Rescale so dont cut off xLabel
set(gca,'OuterPosition',[0.01 0.01 0.99 0.99])

print('-depsc2',resolution_str, filename);   % export .eps (In fact, eps does not really care about the resolution)
print('-dpng',resolution_str, filename);     % export .png

savefig(filename); %save the fig for backup







