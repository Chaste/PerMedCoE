
clear; close all;

addpath ../../ChasteNumericalMethods/MatlabScripts/

NumVoxs = [4, 8, 12, 16, 32, 64];
cells_x = [-10,-10, 10, 10,-30,-30, 30, 30, 10,-10, 10,-10];
cells_y = [ 10,-10, 10,-10,-10, 10, 10,-10, 30, 30,-30,-30];

figure
hold on

has_cells = false;

base_dir = 'TestDiffusionConvergence_no_cells/';

if has_cells
    base_dir = 'TestDiffusionConvergence_with_cells/';
end

for NumVoxIndex = 1:length(NumVoxs)
    NumVox = NumVoxs(NumVoxIndex);

    Data{NumVoxIndex} = readmatrix(['../../../testoutput/',base_dir,num2str(NumVox),'.dat'],'NumHeaderLines',1);

    plot(Data{NumVoxIndex}(:,1),Data{NumVoxIndex}(:,2))
end
legend("N=4","N=8","N=12","N=16","N=32","N=64", 'location','northeast')
title("Concentration at Centre")
xlabel("t")
ylabel("Concentration")
%axis([0,10,0,10]);
if has_cells
    SaveAsPngEpsAndFig(-1,'CentreConcentrationOverTime_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'CentreConcentrationOverTime_no_cells', 12, 7/5, 12)
end

for NumVoxIndex = 1:length(NumVoxs)-1
    error(NumVoxIndex) = abs(Data{NumVoxIndex}(end,2)-Data{end}(end,2));
end

figure
loglog(NumVoxs(1:end-1),error, 'kx-');
xlabel("N")
ylabel("Error")
title("Error Convergence")
%axis([0.9*4,1.1*32,10^(-1.5),1]);

if has_cells
    SaveAsPngEpsAndFig(-1,'ErrorConvergence_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'ErrorConvergence_no_cells', 12, 7/5, 12)
end

for NumVoxIndex = 1:length(NumVoxs)
    figure
    NumVox = NumVoxs(NumVoxIndex);

    dir = [base_dir,  num2str(NumVox)];

    pde_data = load(['../../../testoutput/',dir,'/txt_output/results_Variable_0_1000.txt']);

    width = sqrt(length(pde_data));

    C=reshape(pde_data,[width,width]);

    x=linspace(-120,120,width);
    [X,Y]= meshgrid(x,x);

    surf(X,Y,-C,'EdgeColor','k','FaceColor','interp')
    view(2)
 
    
    title(['N=',num2str(NumVox)])
    if has_cells
        colorbar('Ticks',[-10,-9.5,-9,-8.5,-8],'TickLabels',{'10','9.5','9','8.5','8'})
        clim([-10,-8.5])
        circles(cells_x, cells_y, 10*ones(size(cells_x)),'FaceColor','none','linewidth',1);
        axis equal
        SaveAsPngEpsAndFig(-1,['CellBasedUptake',num2str(NumVox)], 12, 7/5, 12);
    else
        colorbar('Ticks',[-10,-9,-8,-7,-6,-5],'TickLabels',{'10','9','8','7','6','5'})
        clim([-10,-5])
        axis equal
        SaveAsPngEpsAndFig(-1,['UniformUptake',num2str(NumVox)], 12, 7/5, 12);
    end

end 
