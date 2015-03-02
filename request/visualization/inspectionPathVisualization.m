%% inspectionPathVisualization.m
% 
% This script was written by Andreas Bircher on 6th October 2014
%         andreas.bircher@mavt.ethz.ch / bircher@gmx.ch
% 
% Scenarios for autonomous inspection are visualized, for which the path is
% computed using the provided planner.
% 
%% 
clear all; close all;
run('inspectionScenario');

handle = figure;
plot3(inspectionPath(:,1),inspectionPath(:,2),inspectionPath(:,3),'b', 'LineWidth', 2.0,'LineSmoothing', 'on');
hold on;
quiver3(inspectionPath(:,1),inspectionPath(:,2),inspectionPath(:,3),3*cos(inspectionPath(:,6)),3*sin(inspectionPath(:,6)),zeros(size(inspectionPath(:,1))),'g')
patch(meshX',meshY',meshZ','r','EdgeColor','k','FaceAlpha',0.7);
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
if(numObstacles>0)
    for i = 1:numObstacles
        setBox(handle, obstacle{i}(2,1), obstacle{i}(2,2), obstacle{i}(2,3), obstacle{i}(1,1), obstacle{i}(1,2), obstacle{i}(1,3));
    end
end
hold off;
title(['Inspection path with cost ', num2str(inspectionCost), 's']);
axis equal;

