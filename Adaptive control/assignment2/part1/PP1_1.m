%% Clear everything
clear;clc;close all
global drawPlot;
drawPlot = -1;
run('PP1_0.m');
drawPlot = 1;

fprintf('Roots of continues transfer function:\n');
continuesRoots = roots(G_Original.den{1});
display(continuesRoots);
figure;
pzplot(G_Original);
ee = findobj(gca,'type','line');
for i = 1:length(ee)
    set(ee(i),'markersize',24); 
    set(ee(i), 'linewidth',2) ; 
end
xlim([-2 1]);title('Pole-Zero map of continues system');fontsize( 24 ,"points");

fprintf('Roots of discrete transfer function:\n');
discreteRoots = roots(Gz.den{1});
display(discreteRoots);
figure;
pzplot(Gz);
ee = findobj(gca,'type','line');
for i = 1:length(ee)
    set(ee(i),'markersize',24); 
    set(ee(i), 'linewidth',2) ; 
end
title('Pole-Zero map of discrete system');fontsize( 24 ,"points");
