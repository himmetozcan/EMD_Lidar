% Morphological Empirical Mode Decomposition (EMD) application for 
% LiDAR based DSM filtering. Inpainting is used for upper and lower
% envelope estimation.
% inputs
%       im: N x M input image to be decomposed
%       threshold_extrema: elevation threshold (m) for extrema detection
%       morphOpenSize_extrema: window size for extrema detection
%       costF: Cost function threshold for stopping IMF extraction in EMD
%       imfMaxNum: Maximum imf number
%  output
%       imfs: N x M x imfMaxNum

% Author: Abdullah H. Ozcan
% e-mail: himmetozcan@gmail.com
% 11/03/16

%%
function imfs = emd_morph(im, threshold_extrema, morphOpenSize_extrema, costF, imfMaxNum)

im = double(im);

h_f = im;
k = 1;
[n, m] = size(im);
imfs = zeros(n, m, imfMaxNum);

while(k < imfMaxNum)
    [imf_temp, residue_temp] = sifting(h_f, threshold_extrema, morphOpenSize_extrema, costF);
    imfs(:, :, k) = imf_temp;
    k = k + 1;
    h_f = residue_temp;
end

imfs(:,:,k) = residue_temp;

end

%% Sifting process
function [ h_imf, residue ] = sifting(im, threshold_extrema, morphOpenSize_extrema, costF)

input_image_temp = im; % keep the original
counter = 1; % initialize the counter
costPrev = 2; % initialize with a high cost function value

while(1)
    
    % Morphological tophat transform for extrema detection
    tmp = imtophat(input_image_temp,strel('disk',morphOpenSize_extrema));
    
    % Lower envelope estimation
    dets = abs(tmp) > threshold_extrema;
    tmp_dsm = input_image_temp;
    tmp_dsm(dets) = nan;
    zmin = inpaint_nans(tmp_dsm, 4);
    
    % Upper envelope estimation
    dets2 = abs(tmp) <= threshold_extrema;
    tmp_dsm = input_image_temp;
    tmp_dsm(dets2) = nan;
    zmax = inpaint_nans(tmp_dsm, 4);
    
    % Mean envelope estimation
    zavggrid = (zmax + zmin) / 2;
    ind = find(zmin > zmax);
    zavggrid(ind) = zmin(ind);
    
    % IMF check
    h_imf = input_image_temp - zavggrid;
    eps = 0.00000001;
    num = sum(sum((h_imf-input_image_temp).^2));
    den = sum(sum((input_image_temp).^2)) + eps;
    costFunction = num/den;
    
    % Loop check
    if counter==4 || costFunction>2 || (costPrev-costFunction)<0 || costFunction<costF
        break;
    else
        input_image_temp = h_imf;
    end
    
    counter = counter + 1;
    costPrev = costFunction;
end

% Residue
residue = im - h_imf;

end

