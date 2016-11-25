%% Compile the pcl interface to mex file (Linux)
mex -v pcl_transform.cpp pcl_mex_conversions.cpp ...
    -L/usr/lib/ ...
    -lpcl_common ...
    -I/usr/include/pcl-1.8/pcl/ ...
    -I/usr/include/pcl-1.8/ ...
    -I/usr/include/eigen3/ ...
    -I/usr/include/
% NOTE: the matlab startup script modifies LD_LIBRAY_PATH which leads to an
% old version of glibc being used and leads to errors in linking process.
% This can be circumvented with the following measures:
% 1. (using bash) matlab needs to be run with:
%  LD_PRELOAD="/usr/lib/libstdc++.so.6" matlab
% 2. symlink libraries in $MATLABROOT/TODO to system libraries

%% Compile the pcl interface to mex file (Windows)
% NOTE: requires pcl standalone (with all dependencies)
% Alternative: use visual studio project(see readme)
mex -v pcl_transform.cpp pcl_mex_conversions.cpp ...
    -L"C:\Program Files\PCL 1.6.0\lib" ...
    -lpcl_common_release ...
    -I"C:\Program Files\PCL 1.6.0\include\pcl-1.6" ...
    -I"C:\Program Files\PCL 1.6.0\include\pcl-1.6\pcl" ...
    -I"C:\Program Files\PCL 1.6.0\3rdParty\Eigen\include" ...
    -I"C:\Program Files\PCL 1.6.0\3rdParty\Boost\include"
   
%% run the mex file
cloud = zeros(5/0.1 + 1, 3);
cloud(:,1) = 0:0.1:5;
theta = -pi/2;
rotAroundX = [cos(theta)  sin(theta)  0 0;
              -sin(theta) cos(theta)  0 0;
              0           0           1 0;
              0           0           0 1];

cloud_rotated = pcl_transform(cloud, rotAroundX);

pcshow([cloud; cloud_rotated], 'MarkerSize', 100); %TODO not working with LD_PRELOAD
