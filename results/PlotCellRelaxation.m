 
clear; close all;

base_dir = '../../../output/PerMedCoE/TestCellRelaxation/';

position_data = load([base_dir,'/results_from_time_0/node_locations.dat']);



plot(position_data(:,1),10*(position_data(:,5)-position_data(:,2)),'kx')
hold on 
plot(position_data([1,end],1),10*[1,1],'k:')
axis([0,10,8.9,10.1])

title("Chaste Force Cell Separation")
ylabel("Separation (um)")
xlabel("Time (min)")
SaveAsPngEpsAndFig(-1,'HertzCellRelaxation', 12, 7/5, 12)


figure 

base_dir = '../../../output/PerMedCoE/TestCellHertzRelaxation/';

position_data = load([base_dir,'/results_from_time_0/node_locations.dat']);



plot(position_data(:,1)/3600,(position_data(:,5)-position_data(:,2)),'kx')
hold on 
plot(position_data([1,end],1)/3600,10^(-5)*[1,1],'k:')
axis([0,3,8.9*10^(-6),10.1*10^(-6)])





title("Hertz Cell Separation")
ylabel("Separation (m)")
xlabel("Time (hours)")
SaveAsPngEpsAndFig(-1,'HertzCellRelaxation', 12, 7/5, 12)

