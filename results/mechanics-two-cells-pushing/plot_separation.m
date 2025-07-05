clear 
close all

load('../../../../testoutput/TestMechanicsPushing/results_from_time_0/results.viznodelocations')
%load('results.viznodelocations')

separation = results(:,5)-results(:,2)

plot(results(:,1)*60,separation,'r.')
hold on
plot(results(:,1)*60,separation,'r-')

plot(results(:,1)*60,1*ones(size(results(:,1))),'k--')
plot(results(:,1)*60,0.9*ones(size(results(:,1))),'k--')