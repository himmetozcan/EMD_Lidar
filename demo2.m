clear
close all
clc

%% Parameters

par.cellSize = 0.5; % spatial resolution in meters

par.iterationNum = 10; % iteration number
par.maxW = 25/par.cellSize; % maximum window size for extrema detection (meters)
par.maxThr = 3; % maximum threshold for extrema detection (meters)
par.te = 1; % hard elevation threshold  (meters)

%%  Get data

datadirectory = 'utah-x1';
fileName = '\points.las';
filename = [datadirectory, fileName];

[s, h, v] = lasread(filename, 'xyzc');

Xp(:,1) = s.X;
Xp(:,2) = s.Y;
Xp(:,3) = s.Z;

% load true DTM
load([datadirectory, '\trueDTM.mat'])

%% EMD based lidar filtering

[Gdsm, Gdtm, Pdtm, Pobjects, R] = filterlidar_emd(Xp, par);

%% Performance

disp('Calculating performance ...')

Gobjects = (Gdsm-Gdtm) > par.te; % Detections above te
trueResults = (Gdsm-trueDTM) > par.te; % True detections above te

performances=calculateperformances(Gobjects(:), trueResults(:));
disp(' ')
disp(['Kappa(%): ', num2str(performances.kappa)])
disp(['Total Error(%): ', num2str(performances.TE), ', Type-I Error(%): ', num2str(performances.TI),', Type-II Error(%): ', num2str(performances.TII)])

%% Visual results

[xi, yi] = ir2xiyi(Gdsm,R);
[XI, YI] = meshgrid(xi,yi);

v1=26; v2=32;

figure; surf(XI,YI,Gdsm,hillshade2(Gdsm),'edgecolor','none'); axis equal vis3d; colormap gray;
view(v1,v2)
zlim([min(Gdsm(:)), max(Gdsm(:))])
title('DSM')

figure; surf(XI,YI,Gdtm,hillshade2(Gdtm),'edgecolor','none'); axis equal vis3d; colormap gray;
view(v1,v2)
zlim([min(Gdsm(:)), max(Gdsm(:))])
title('Generated DTM')

figure; surf(XI,YI,Gdtm,hillshade2(trueDTM),'edgecolor','none'); axis equal vis3d; colormap gray;
view(v1,v2)
zlim([min(Gdsm(:)), max(Gdsm(:))])
title('True DTM')

figure,imagesc(xi,yi,Gobjects),axis image;axis xy;colormap gray
title('Detected Objects')

