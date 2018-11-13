function dsm = getdsm(x,y,z,cellSize,outlierThr)


%% DSM generation and Outlier Removal

%-DSM Generation
dsm=createDSM(x,y,z,1/cellSize,1/cellSize); % DSM as IMAGE
% dsm=flipud(dsm);

%- Remove outliers in DSM
outlierWin=5; 
% outlierThr=5;
rdsm=medfilt2(dsm,[outlierWin outlierWin],'symmetric');
tmp=dsm-rdsm;
ind_outlier=find(abs(tmp)>outlierThr);
dsm(ind_outlier)=rdsm(ind_outlier);
