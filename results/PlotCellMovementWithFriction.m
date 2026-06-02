 
clear; close all;

addpath ../../ChasteNumericalMethods/MatlabScripts/

base_dir = '../../../testoutput/PerMedCoE/TestCellRelaxation/';

position_data = load([base_dir,'/results_from_time_0/node_locations.dat']);



plot(position_data(:,1),10*(position_data(:,5)-position_data(:,2)),'kx')
hold on 
plot(position_data([1,end],1),10*[1,1],'k:')
axis([0,10,8.9,10.1])

title("Cell Separation")
ylabel("Separation (um)")
xlabel("Time (min)")
SaveAsPngEpsAndFig(-1,'CellRelaxation', 12, 7/5, 12)

