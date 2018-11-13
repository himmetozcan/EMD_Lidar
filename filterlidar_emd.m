function [Gdsm, Gdtm, Pdtm, Pobjects, R] = filterlidar_emd(Xp, par)

%"LiDAR Data Filtering and DTM Generation using Empirical Mode Decomposition"
% (Matlab code for iterative morphology based EMD algorithm for detecting
% objects and generating DTM from LiDAR Data)

% input
%       Xp  : 'N x 3' (x,y,z) LiDAR points each with length N
%       par : structure input for parameters 
% output
%       Gdsm     : generated grid DSM
%       Gdtm     : generated grid DTM
%       Pdtm     : generated point DTM
%       Pobjects : generated point binary object class 1: object, 0: ground
%       R        : spatial reference matrix
% Author: Abdullah H. Ozcan
% e-mail: himmetozcan@gmail.com
% 11/03/16

%% parameters

try
   slopeThr = par.slopeThr;
catch
   slopeThr = false; % use slope thresholding "true" or "false"
end

try
    outlierThr = par.outlierThr;
catch
   outlierThr = 30; % remove outliers above x meters for dsm generation 
end

cellSize = par.cellSize;
iterationNum = par.iterationNum;
maxW = par.maxW;
maxThr = par.maxThr;
te = par.te;

imfMaxNum = 2; % Maximum imf number ("2": 1 imf + 1 residue, "3": 2 imfs + 1 residue and so on ...)
costF = 0.25; % Cost function threshold for stopping IMF extraction [0,1].

ws = round(linspace(1, maxW, iterationNum));
thrs = linspace(0.1, maxThr, iterationNum);

%% DSM Generation

x=Xp(:,1);
y=Xp(:,2);
z=Xp(:,3);

disp('Generating grid DSM ...')
dsm = getdsm(x,y,z,cellSize,outlierThr);

xi = ceil(min(x)):cellSize:floor(max(x));
yi = floor(max(y)):-cellSize:ceil(min(y));
R = makerefmat(xi(1),yi(1),xi(2) - xi(1),yi(2) - yi(1));

%% Iterative EMD

disp('Iterative EMD process ...')

objects=zeros(size(dsm));
parfor i=1:length(ws)
    disp([length(ws)-i+1])
    
    threshold_extrema = thrs(i);
    morphOpenSize_extrema = ws(i);
    
    % iterative EMD
    imfs = emd_morph(dsm, threshold_extrema, morphOpenSize_extrema, costF, imfMaxNum);
    
    % Residue (last component)
    residue=imfs(:,:,end);
    
    % Detections
    minuteObjects=(dsm-residue)>(threshold_extrema*0.75);
    
    % Sum all detections
    objects=objects | minuteObjects;
end

%% DTM estimation with inpainting

disp('DTM estimation ...')

dtm = dsm;
dtm(objects) = nan;
dtm = inpaint_nans(dtm, 4);

Gdtm = dtm;
Gdsm = dsm;

%%

disp('Object Detection ...')

[r, c] = map2pix(R, x, y);

Pdtm = interp2(dtm, c, r, 'spline');
[gx, gy] = gradient(dtm);
slopeGridDtm = sqrt(gx.^2 + gy.^2);
slopeDtm = interp2(slopeGridDtm, c, r, 'spline');

useSlope = double(slopeThr);

thr = te + useSlope * abs((slopeDtm).^2);
objects = (z - Pdtm) > thr;
Pobjects = objects;




