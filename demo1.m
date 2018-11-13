clear
close all
clc

%% Parameters

par.cellSize = 1; % spatial resolution in meters
par.outlierThr = 5; % remove outliers above x meters for dsm generation  

par.iterationNum = 10; % iteration number
par.maxW = 20/par.cellSize; % maximum window size for extrema detection (meters)
par.maxThr = 4; % maximum threshold for extrema detection (meters)
par.te = 0.6; % hard elevation threshold  (meters)

par.slopeThr = true; % use slope thresholding "true" or "false"

%%  Get data

datadirectory='isprs';

fileName='\samp11.txt';
% fileName='\samp12.txt';
% fileName='\samp54.txt';

Xp = dlmread([datadirectory,fileName]);

trueResults = Xp(:,4);  % 0 is Ground, 1 is Object

%% EMD based lidar filtering

[Gdsm, Gdtm, Pdtm, Pobjects, R] = filterlidar_emd(Xp, par);

%% Performance

disp('Calculating performance ...')

performances=calculateperformances(Pobjects(:), trueResults(:));
disp(' ')
disp(['Kappa(%): ', num2str(performances.kappa)])
disp(['Total Error(%): ', num2str(performances.TE), ', Type-I Error(%): ', num2str(performances.TI),', Type-II Error(%): ', num2str(performances.TII)])

%% Visual results

[xi, yi] = ir2xiyi(Gdsm,R);
[XI, YI] = meshgrid(xi,yi);
Gobjects = (Gdsm-Gdtm) > par.te; % Detections above te

v1=66; v2=36; 

figure; surf(XI,YI,Gdsm,hillshade2(Gdsm),'edgecolor','none'); axis equal vis3d; colormap gray;
view(v1,v2)
zlim([min(Gdsm(:)), max(Gdsm(:))])
title('DSM')

figure; surf(XI,YI,Gdtm,hillshade2(Gdtm),'edgecolor','none'); axis equal vis3d; colormap gray;
view(v1,v2)
zlim([min(Gdsm(:)), max(Gdsm(:))])
title('Generated DTM')

figure,imagesc(xi,yi,Gobjects),axis image;axis xy;colormap gray
title('Detected Objects')

