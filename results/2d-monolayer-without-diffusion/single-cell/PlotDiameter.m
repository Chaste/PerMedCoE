clear 
close all

load('../../../../../testoutput/2DMonolayerWithoutDiffusion/SingleCell/results_from_time_0/tissuewidth.dat')


plot(4+tissuewidth(:,1)/24,tissuewidth(:,4),'r.')
hold on
plot(14+[0,50,72,145,170,310]/24,[1140,1400,1590,2040,2250,3040],'k-o')

%figure
%plot(tissuewidth(:,1)/24,tissuewidth(:,5),'r.')
