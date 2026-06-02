
clear; close all;

addpath ../../ChasteNumericalMethods/MatlabScripts/

cells_x = [-10,-10, 10, 10,-30,-30, 30, 30, 10,-10, 10,-10];
cells_y = [ 10,-10, 10,-10,-10, 10, 10,-10, 30, 30,-30,-30];
cells_z = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

NumVoxs = [6, 12, 24, 48];

% figure
% hold on

has_cells = true;

base_dir = 'TestDiffusionConvergence3d_no_cells/';

if has_cells
    base_dir = 'TestDiffusionConvergence3d_with_cells/';
end

for NumVoxIndex = 1:length(NumVoxs)
    figure
    NumVox = NumVoxs(NumVoxIndex);

    dir = [base_dir,  num2str(NumVox)];

    pde_data = load(['../../../testoutput/',dir,'/results_from_time_0/results.vizpdesolution_oxygen']);

    
    for t=1:length(pde_data(:,1))
        time{NumVoxIndex}(t)= pde_data(t,1);
        num_cells = length(pde_data(t,2:end))/5;
        index = pde_data(t,2:5:end-4);
        x = pde_data(t,3:5:end-3);
        y = pde_data(t,4:5:end-2);
        z = pde_data(t,5:5:end-1);
        c = pde_data(t,6:5:end);

       
        
        average_c{NumVoxIndex}(t) = mean(c);
        centre_c{NumVoxIndex}(t) = c(floor(length(y)/2)+1);
        
        % Checking in centre
        assert(x(floor(length(y)/2)+1)==0);
        assert(y(floor(length(y)/2)+1)==0);
        assert(z(floor(length(y)/2)+1)==0);


    end

    % Plot end distribution
    z_plane_index = find(z==0);
    
    local_x = x(z_plane_index);
    local_y = y(z_plane_index);
    local_c = c(z_plane_index);
    
    width = sqrt(length(local_x));

    C=reshape(local_c,[width,width]);
    
    xlin=linspace(-120,120,width);
    [X,Y]= meshgrid(xlin,xlin);

    surf(X,Y,-C,'EdgeColor','k','FaceColor','interp')
    view(2)

    title(['N=',num2str(NumVox)])
    if has_cells
        colorbar('Ticks',[-10,-9.5,-9,-8.5,-8, -7.5,-7,-6.5, -6, -5.5, -5],'TickLabels',{'10','9.5','9','8.5','8', '7.5', '7', '6.5','6', '5.5', '5'})
        %clim([-10,-7])
        circles(cells_x, cells_y, 10*ones(size(cells_x)),'FaceColor','none','linewidth',1);
        axis equal
        SaveAsPngEpsAndFig(-1,['TissueCellBasedUptake',num2str(NumVox)], 12, 7/5, 12);
    else
        colorbar('Ticks',[-10,-9,-8,-7,-6,-5],'TickLabels',{'10','9','8','7','6','5'})
        clim([-10,-5])
        axis equal
        SaveAsPngEpsAndFig(-1,['TissueUniformUptake',num2str(NumVox)], 12, 7/5, 12);
    end

end 


%% Plot centre concentration
figure
hold on

for NumVoxIndex = 1:length(NumVoxs)
    plot(time{NumVoxIndex},centre_c{NumVoxIndex})
end

exact_time = [ 0.          0.52631579  1.05263158  1.57894737  2.10526316  2.63157895 ...
  3.15789474  3.68421053  4.21052632  4.73684211  5.26315789  5.78947368 ...
  6.31578947  6.84210526  7.36842105  7.89473684  8.42105263  8.94736842 ...
  9.47368421 10.        ];

exact_centre = [10.          7.83908758  7.49652531  7.34117386  7.25600623  7.20713235 ...
  7.17877842  7.16228652  7.15268824  7.14710124  7.14384902  7.14195589 ...
  7.14085387  7.14021238  7.13983896  7.13962159  7.13949506  7.1394214 ...
  7.13937852  7.13935357];

plot(exact_time,exact_centre,'k:')

legend("N=6","N=12","N=24","N=48", 'location','northeast')
title("Concentration at Centre")
xlabel("t")
ylabel("Concentration")
%axis([0,10,0,10]);
if has_cells
    SaveAsPngEpsAndFig(-1,'TissueCentreConcentrationOverTime_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'TissueCentreConcentrationOverTime_no_cells', 12, 7/5, 12)
end

%error
for NumVoxIndex = 1:length(NumVoxs)
    error_centre(NumVoxIndex) = abs(centre_c{NumVoxIndex}(end)-exact_centre(end));
end

figure
loglog(NumVoxs,error_centre, 'kx-');
xlabel("N")
ylabel("Error")
title("Centre Error Convergence")
%axis([0.9*4,1.1*32,10^(-1.5),1]);

if has_cells
    SaveAsPngEpsAndFig(-1,'TissueCenterErrorConvergence_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'TissueCentreErrorConvergence_no_cells', 12, 7/5, 12)
end

%% Average concentration
figure
hold on

for NumVoxIndex = 1:length(NumVoxs)
    plot(time{NumVoxIndex},average_c{NumVoxIndex})
end


exact_time = [ 0.          0.52631579  1.05263158  1.57894737  2.10526316  2.63157895 ...
  3.15789474  3.68421053  4.21052632  4.73684211  5.26315789  5.78947368 ...
  6.31578947  6.84210526  7.36842105  7.89473684  8.42105263  8.94736842 ...
  9.47368421 10.        ];
exact_average = [10.          9.92815197  9.87126486  9.83518508  9.81375256  9.80121673 ...
  9.79391127  9.78965756  9.78718127  9.78573979  9.78490068  9.78441223 ...
  9.78412789  9.78396238  9.78386603  9.78380995  9.7837773   9.7837583 ...
  9.78374724  9.7837408 ];

plot(exact_time,exact_average,'k:')
%plot([0,time{1}(end)],2.7692*[1,1],'k:')
legend("N=6","N=12","N=24","N=48",'location','northeast')
title("Average Concentration")
xlabel("t")
ylabel("Concentration")
%axis([0,10,0,10]);
if has_cells
    SaveAsPngEpsAndFig(-1,'TissueAverageConcentrationOverTime_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'TissueAverageConcentrationOverTime_no_cells', 12, 7/5, 12)
end


for NumVoxIndex = 1:length(NumVoxs)
    error_average(NumVoxIndex) = abs(average_c{NumVoxIndex}(end)-exact_average(end));
end

figure
loglog(NumVoxs,error_average, 'kx-');
xlabel("N")
ylabel("Error")
title("Average Error Convergence")
%axis([0.9*4,1.1*32,10^(-1.5),1]);

if has_cells
    SaveAsPngEpsAndFig(-1,'TissueAverageErrorConvergence_with_cells', 12, 7/5, 12)
else
    SaveAsPngEpsAndFig(-1,'TissueAverageErrorConvergence_no_cells', 12, 7/5, 12)
end


%% Save data to dat file

%Headder
% time(minutes) centre_concentration(uM) average_concentration(uM)

for NumVoxIndex = 1:length(NumVoxs)
    local_data = [time{NumVoxIndex}', centre_c{NumVoxIndex}', average_c{NumVoxIndex}'];
    fname = string([num2str(NumVoxs(NumVoxIndex)),'.txt'])
    save(fname,'local_data',"-ascii");
end