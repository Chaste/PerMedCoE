 
clear; close all;

addpath ../../ChasteNumericalMethods/MatlabScripts/

base_dir = '../../../testoutput/PerMedCoE/TestCellMovementWithFriction/';

position_data = load([base_dir,'/results_from_time_0/node_locations.dat']);


velocity_data = load([base_dir,'/results_from_time_0/cell_velocities.dat']);


plot(position_data(:,1),position_data(:,2),'kx')
hold on 
plot([0,12],8.33333*[1,1],'k:')

title("Cell Position")
ylabel("Position (um)")
xlabel("Time (s)")
SaveAsPngEpsAndFig(-1,'PositionCellMovementWithFriction', 12, 7/5, 12)


figure
 
plot(velocity_data(:,1),velocity_data(:,5),'kx')
title("Cell Velocity")
ylabel("Velocity (um/s)")
xlabel("Time (s)")
SaveAsPngEpsAndFig(-1,'VelocityCellMovementWithFriction', 12, 7/5, 12)

