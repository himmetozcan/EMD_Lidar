function ref_dsm=createDSM(x,y,z,step1,step2)
warning('off')
% step1=1;
% step2=1.5;
F = TriScatteredInterp(x,y,z,'nearest');
ti = ((min(x)):1/step1:(max(x)));
tj = ((min(y)):1/step2:(max(y)));
% ti = ((min(x)):step1:(max(x)));
% tj = ((min(y)):step2:(max(y)));
[qx,qy] = meshgrid(ti,tj);
ref_dsm = F(qx,qy);
% ref_dsm(ref_dsm>1000)=1000;
% figure;imagesc(ref_dsm);axis image

ref_dsm=flipud(ref_dsm);