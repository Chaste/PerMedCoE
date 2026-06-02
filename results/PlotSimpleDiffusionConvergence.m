
clear; close all;

addpath ../../ChasteNumericalMethods/MatlabScripts/

NumVoxs = [6, 12, 24, 48];

cells_x = [-10,-10, 10, 10,-30,-30, 30, 30, 10,-10, 10,-10];
cells_y = [ 10,-10, 10,-10,-10, 10, 10,-10, 30, 30,-30,-30];
cells_z = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

has_cells = true;

base_dir = 'TestSimpleDiffusionConvergence3d_no_cells/';

solution_centre = [ 0.          0.52631579  1.05263158  1.57894737  2.10526316  2.63157895  3.15789474  3.68421053  4.21052632  4.73684211  5.26315789  5.78947368  6.31578947  6.84210526  7.36842105  7.89473684  8.42105263  8.94736842  9.47368421  10.0;        
                   10.          5.93968034  3.94462468  3.18079231  2.91133497  2.81811149  2.78600674  2.77496231  2.77116386  2.76985756  2.76940833  2.76925384  2.76920071  2.76918244  2.76917616  2.76917399  2.76917325  2.769173    2.76917291  2.76917288];

solution_average = [ 0.          0.52631579  1.05263158  1.57894737  2.10526316  2.63157895  3.15789474  3.68421053  4.21052632  4.73684211  5.26315789  5.78947368  6.31578947  6.84210526  7.36842105  7.89473684  8.42105263  8.94736842  9.47368421  10.0 ;
                    10.          7.79217467  7.1711064   6.96508102  6.89482579  6.8707134   6.86242517  6.85957518  6.8585951   6.85825806  6.85814215  6.85810229  6.85808858  6.85808387  6.85808224  6.85808169  6.8580815   6.85808143  6.85808141  6.8580814];


if has_cells
    base_dir = 'TestSimpleDiffusionConvergence3d_with_cells/';

    solution_centre = [ 0.          0.52631579  1.05263158  1.57894737  2.10526316  2.63157895   3.15789474  3.68421053  4.21052632  4.73684211  5.26315789  5.78947368  6.31578947  6.84210526  7.36842105  7.89473684  8.42105263  8.94736842  9.47368421 10.        ;
                       10.          7.83908758  7.49652531  7.34117386  7.25600623  7.20713235  7.17877842  7.16228652  7.15268824  7.14710124  7.14384902  7.14195589  7.14085387  7.14021238  7.13983896  7.13962159  7.13949506  7.1394214  7.13937852  7.13935357];
        
    solution_average = [ 0.          0.52631579  1.05263158  1.57894737  2.10526316  2.63157895  3.15789474  3.68421053  4.21052632  4.73684211  5.26315789  5.78947368  6.31578947  6.84210526  7.36842105  7.89473684  8.42105263  8.94736842  9.47368421 10.        ;
                        10.          9.92815197  9.87126486  9.83518508  9.81375256  9.80121673  9.79391127  9.78965756  9.78718127  9.78573979  9.78490068  9.78441223  9.78412789  9.78396238  9.78386603  9.78380995  9.7837773   9.7837583  9.78374724  9.7837408 ];
end

%% Centre Concentration
figure
hold on

for NumVoxIndex = 1:length(NumVoxs)
    NumVox = NumVoxs(NumVoxIndex);

    Data{NumVoxIndex} = readmatrix(['../../../testoutput/',base_dir,num2str(NumVox),'.dat'],'NumHeaderLines',1);

    plot(Data{NumVoxIndex}(:,1),Data{NumVoxIndex}(:,2))
end

plot(solution_centre(1,:),solution_centre(2,:), 'k:')
legend("N=6","N=12","N=24","N=48", 'location','northeast')
title("Concentration at Centre")
xlabel("t")
ylabel("Concentration")
%axis([0,10,0,10]);
if has_cells
    SaveAsPngEpsAndFig(-1,'SimpleCentreConcentrationOverTime_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'SimpleCentreConcentrationOverTime_no_cells', 12, 7/5, 12)
end

for NumVoxIndex = 1:length(NumVoxs)
    error(NumVoxIndex) = abs(Data{NumVoxIndex}(end,2)-solution_centre(end,end));
end

figure
loglog(NumVoxs(1:end),error, 'kx-');
xlabel("N")
ylabel("Error")
title("Centre Error Convergence")
%axis([0.9*4,1.1*32,10^(-1.5),1]);

if has_cells
    SaveAsPngEpsAndFig(-1,'SimpleCenterErrorConvergence_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'SimpleCentreErrorConvergence_no_cells', 12, 7/5, 12)
end

%% Average Concentration

figure
hold on 
for NumVoxIndex = 1:length(NumVoxs)
    plot(Data{NumVoxIndex}(:,1),Data{NumVoxIndex}(:,3))
end

plot(solution_average(1,:),solution_average(2,:),'k:')
legend("N=6","N=12","N=24","N=48", 'location','northeast')
title("Average Concentration")
xlabel("t")
ylabel("Concentration")
%axis([0,10,0,10]);
if has_cells
    SaveAsPngEpsAndFig(-1,'SimpleAverageConcentrationOverTime_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'SimpleAverageConcentrationOverTime_no_cells', 12, 7/5, 12)
end



for NumVoxIndex = 1:length(NumVoxs)
    error(NumVoxIndex) = abs(Data{NumVoxIndex}(end,3)-solution_average(end,end));
end

figure
loglog(NumVoxs(1:end),error, 'kx-');
xlabel("N")
ylabel("Error")
title("Average Error Convergence")
%axis([0.9*4,1.1*32,10^(-1.5),1]);

if has_cells
    SaveAsPngEpsAndFig(-1,'SimpleAverageErrorConvergence_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'SimpleAverageErrorConvergence_no_cells', 12, 7/5, 12)
end

for NumVoxIndex = 1:length(NumVoxs)
    figure
    NumVox = NumVoxs(NumVoxIndex);

    dir = [base_dir,  num2str(NumVox)];

    pde_data = load(['../../../testoutput/',dir,'/txt_output/results_Variable_0_200.txt']);

    width = nthroot(length(pde_data),3);

    C=reshape(pde_data,[width,width,width]);

    x=linspace(-120,120,width);
    [X,Y]= meshgrid(x,x);

    surf(X,Y,-C(:,:,ceil(width/2)),'EdgeColor','k','FaceColor','interp')
    view(2)
 
    
    title(['N=',num2str(NumVox)])
    if has_cells
        colorbar('Ticks',[-10,-8,-6,-4,-2],'TickLabels',{'10','8','6','4','2'})
        clim([-10,-4])
        circles(cells_x, cells_y, 10*ones(size(cells_x)),'FaceColor','none','linewidth',1);
        axis equal
        SaveAsPngEpsAndFig(-1,['CellBasedUptake',num2str(NumVox)], 12, 7/5, 12);
    else
        colorbar('Ticks',[-10,-8,-6,-4,-2],'TickLabels',{'10','8','6','4','2'})
        clim([-10,-2])
        axis equal
        SaveAsPngEpsAndFig(-1,['UniformUptake',num2str(NumVox)], 12, 7/5, 12);
    end

end 
