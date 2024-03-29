clear all, close all, clc
% MM = se3(eul2rotm(deg2rad([180 -90 0]),"zyx"),[0,0,0])
% this transformation will be utilized later, if needed

head_mask = load("head_mask.mat");

head_mask = head_mask.head_mask;

vrt = head_mask.Vertices;

I = find(vrt(:,1)>0.02);

vrt2 = vrt(I,:);

trvec = [.5, -.3, .1];

Ta = se3(eul2rotm(deg2rad([45 15 30])),trvec)

vrt_mod = transform(Ta,vrt2);

%%
Options.Registration = 'Rigid';
Options.TolX = 1e-16;
Options.TolP = 1e-16;
Options.Optimizer = 'lsqnonlin';
Options.Verbose = 0;

[data,Te] = ICP_finite(vrt_mod,vrt2,Options);

disp('only rigid transformation')
RMSE = sqrt(3)*rmse(data(:),vrt_mod(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))


figure(1)
hold on
scatter3(vrt_mod(:,1),vrt_mod(:,2),vrt_mod(:,3))
scatter3(data(:,1),data(:,2),data(:,3))
axis equal, grid minor, rotate3d on
title(['Virtual onto Real with RMSE = ' num2str(RMSE)])
legend('Exact points', 'Estimated point cloud')
xlabel('[m]'),ylabel('[m]'),zlabel('[m]')
campos([2.4666    0.7059   -0.5055])


% noise with 1mm in all axes + downsampled with ratio of 20
vrt_mod3 = vrt_mod(1:20:end,:) + 0.001*randn(size(vrt_mod(1:20:end,:)));
[data,Te] = ICP_finite(vrt_mod3,vrt2,Options);

disp('1mm noise downsampled with 20')
RMSE = sqrt(3)*rmse(reshape(data(1:20:end,:),[],1),vrt_mod3(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

figure(3)
hold on
scatter3(data(:,1),data(:,2),data(:,3))
scatter3(vrt_mod3(:,1),vrt_mod3(:,2),vrt_mod3(:,3),'+'), axis equal, grid minor
rotate3d on
title(['1mm noise downsampled with 20, RMSE = ', num2str(RMSE)])
legend('Estimated point cloud','Measured points')
xlabel('[m]'),ylabel('[m]'),zlabel('[m]')
campos([2.4666    0.7059   -0.5055])


% noise with 1mm in all axes + downsampled with ratio of 50
vrt_mod50 = vrt_mod(1:50:end,:) + 0.001*randn(size(vrt_mod(1:50:end,:)));
[data,Te] = ICP_finite(vrt_mod50,vrt2,Options);

disp('1mm noise downsampled with 50')
RMSE = sqrt(3)*rmse(reshape(data(1:50:end,:),[],1),vrt_mod50(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

figure(6)
hold on
scatter3(data(:,1),data(:,2),data(:,3))
scatter3(vrt_mod50(:,1),vrt_mod50(:,2),vrt_mod50(:,3),'LineWidth',2), axis equal, grid minor
rotate3d on
title(['1mm noise downsampled with 50, RMSE = ' num2str(RMSE)])
legend('Estimated point cloud','Measured points')
xlabel('[m]'),ylabel('[m]'),zlabel('[m]')
campos([2.4666    0.7059   -0.5055])

%% computations without graphs

% noise with 1mm in all axes + downsampled with ratio of 20
vrt_mod20 = vrt_mod(1:20:end,:) + 0.001*randn(size(vrt_mod(1:20:end,:)));
[data,Te] = ICP_finite(vrt_mod20,vrt2,Options);
disp('1mm noise downsampled with 20')
RMSE = sqrt(3)*rmse(reshape(data(1:20:end,:),[],1),vrt_mod20(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

% noise with 1mm in all axes + downsampled with ratio of 50
vrt_mod50 = vrt_mod(1:50:end,:) + 0.001*randn(size(vrt_mod(1:50:end,:)));
[data,Te] = ICP_finite(vrt_mod50,vrt2,Options);
disp('1mm noise downsampled with 50')
RMSE = sqrt(3)*rmse(reshape(data(1:50:end,:),[],1),vrt_mod50(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))


% % % more noise

% noise with 2mm in all axes + downsampled with ratio of 20
vrt_mod20 = vrt_mod(1:20:end,:) + 0.002*randn(size(vrt_mod(1:20:end,:)));
[data,Te] = ICP_finite(vrt_mod20,vrt2,Options);
disp('2mm noise downsampled with 20')
RMSE = sqrt(3)*rmse(reshape(data(1:20:end,:),[],1),vrt_mod20(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

% noise with 2mm in all axes + downsampled with ratio of 50
vrt_mod50 = vrt_mod(1:50:end,:) + 0.002*randn(size(vrt_mod(1:50:end,:)));
[data,Te] = ICP_finite(vrt_mod50,vrt2,Options);
disp('2mm noise downsampled with 50')
RMSE = sqrt(3)*rmse(reshape(data(1:50:end,:),[],1),vrt_mod50(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

% % % % % 3mm noise

% noise with 3mm in all axes + downsampled with ratio of 20
vrt_mod20 = vrt_mod(1:20:end,:) + 0.003*randn(size(vrt_mod(1:20:end,:)));
[data,Te] = ICP_finite(vrt_mod20,vrt2,Options);
disp('3mm noise downsampled with 20')
RMSE = sqrt(3)*rmse(reshape(data(1:20:end,:),[],1),vrt_mod20(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

% noise with 3mm in all axes + downsampled with ratio of 50
vrt_mod50 = vrt_mod(1:50:end,:) + 0.003*randn(size(vrt_mod(1:50:end,:)));
[data,Te] = ICP_finite(vrt_mod50,vrt2,Options);
disp('3mm noise downsampled with 50')
RMSE = sqrt(3)*rmse(reshape(data(1:50:end,:),[],1),vrt_mod50(:))
Teps=Ta*se3(inv(Te))
axang=rotm2axang(Teps.rotm);
rad2deg(axang(4))

%% 
vrt_orig = 1e3*vrt(I,:);
trvec = 1e3*[.5, -.3, .1];
Ta = se3(eul2rotm(deg2rad([45 15 30])),trvec)
vrt_tran = transform(Ta,vrt_orig);

% downsampled pointclouds
vrt2   = vrt_tran(1:2:end,:);
vrt5   = vrt_tran(1:5:end,:);
vrt20  = vrt_tran(1:20:end,:);
vrt50  = vrt_tran(1:50:end,:);
vrt100 = vrt_tran(1:100:end,:);
DS = 5;

% noise ratios
n1=1;
n3=3;
n5=5;
NR=3;

moving = vrt_orig;
for i=1:DS
    for j=1:NR
        if i==1
            static=vrt2;
            dsr=2;
        elseif i==2
            static=vrt5;
            dsr=5;
        elseif i==3
            static=vrt20;
            dsr=20;
        elseif i==4
            static=vrt50;
            dsr=50;
        elseif i==5
            static = vrt100;
            dsr = 100;
        end

        if j==1
            n=n1;
            nr=1;
        elseif j==2
            n=n3;
            nr=3;
        elseif j==3
            n=n5;
            nr=5;
        end

        static = static + n*randn(size(static));

        % algorithm 1
        [data,Te] = ICP_finite(static,moving,Options);
        RMSE = sqrt(3)*rmse(reshape(data(1:dsr:end,:),[],1),static(:));
        Teps=Ta*se3(inv(Te));
        axang=rotm2axang(Teps.rotm);
        thetaeps=rad2deg(axang(4));
        mde10 = 100*axang(4) + norm(Teps.trvec);
        fprintf('Downsampling ratio: %3.0f, Noise ratio: %2.0f[mm], Angular deviation: %3.1f, RMSE: %4.3f, MDE10: %4.3f\n',dsr,nr,thetaeps,RMSE,mde10)

        % algorithm 2
        [Ticp,movingregicp,rmseicp]=pcregistericp(pointCloud(moving),pointCloud(static),'Metric','pointToPoint',...
            'MaxIterations',1e5,'Tolerance',[1e-12,1e-6]);
        Te = se3(Ticp.A);
        Teps = Ta*inv(Te);
        axang=rotm2axang(Teps.rotm);
        thetaeps=rad2deg(axang(4));
        mde10 = 100*axang(4) + norm(Teps.trvec);
        fprintf('Downsampling ratio: %3.0f, Noise ratio: %2.0f[mm], Angular deviation: %3.1f, RMSE: %4.3f, MDE10: %4.3f\n',dsr,nr,thetaeps,rmseicp,mde10)


    end
end
