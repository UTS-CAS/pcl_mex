%% run the mex file

frame1=4;
frame2=7;

filenameSrc=sprintf('/scratch/aalempij/carcass_02/pc_%05d.ply',frame1);
filenameTrg=sprintf('/scratch/aalempij/carcass_02/pc_%05d.ply',frame2);
fprintf(1,'Loading pairs %d and %d\n',frame1,frame2);
ptCloudSrc = pcread(filenameSrc);
ptCloudTrg = pcread(filenameTrg);

cmd_pcl = sprintf('/home/aalempij/Data/svn/bcs/pcl/auto_alignment/build_new/test_normals %s %s',filenameSrc,filenameTrg);
[~,results] = system(cmd_pcl);

ind_str=strfind(results,'Global Transform');
matrix=results((ind_str+16+3):end-1);
matrix = strtrim(matrix);
str=split(matrix);
M=str2double(str);
M=reshape(M,4,4);
M=M';

display(M);

% M(1:3,1:3)=quat2rotm(rotm2quat(M(1:3,1:3)));
% tr=rigid3d(M(1:3,1:3)',M(1:3,4)');

% cloudSrc = [ptCloudSrc.
% cloudSrc = zeros(size(ptCloudTgt,1), 6);
% 
% 
% M2 = pcl_transform(cloud, rotAroundX);
