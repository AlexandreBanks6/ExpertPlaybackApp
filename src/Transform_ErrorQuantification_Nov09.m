clear
clc
close all

DATA_ROOT_CamToScene='..\resources\validation\camToScene';
DATA_ROOT_sTpsm='..\resources\validation\PSMPose';

NDI_T_ARUCO_RIGID_TRANSFORM=[1,0,0,-0.05225;
                            0,0,-1,0.076327;
                            0,1,0,0.048387;
                            0,0,0,1];

% NDI_T_ARUCO_RIGID_TRANSFORM=[0,0,1,-0.076327;
%                             1,0,0,-0.05225;
%                             0,1,0,0.048387;
%                             0,0,0,1];

% NDI_T_ARUCO_RIGID_TRANSFORM=[1,0,0,-0.05225;
%                             0,-1,0,-0.048387;
%                             0,0,-1,0.076327;
%                             0,0,0,1];

%%%%%%%%%%%%%%%%%%%%%%%Cam-To-Scene Transforms%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Reading in data
data_multiple_left=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_multiple_left.csv']);
data_multiple_right=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_multiple_right.csv']);
data_single_left=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_single_left.csv']);
data_single_right=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_single_right.csv']);

[NDI_Mul_left_Angles,NDI_Mul_left_Tanslation,Estim_Mul_Left_Angles,Estim_Mul_Left_Translation]=...
    compareFrameToNDI(data_multiple_left(:,5:16),data_multiple_left(:,19:30),NDI_T_ARUCO_RIGID_TRANSFORM,1);

% Analyzing error
Mul_Left_TransError=NDI_Mul_left_Tanslation-Estim_Mul_Left_Translation;
Mul_Left_TransError=sqrt(sum(Mul_Left_TransError.^2,2));
Mul_Left_TransMean=mean(Mul_Left_TransError);





%%%%%%%%%%%%%%%%%%%%%%%%%lc_T_psm Transforms%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Reading Data
data_PSM1_withErrCorr=readmatrix([DATA_ROOT_sTpsm,'\lc_T_psm_PSM1__withErrCorr.csv']);
data_PSM1_withoutErrCorr=readmatrix([DATA_ROOT_sTpsm,'\lc_T_psm_PSM1__withoutErrCorr.csv']);
data_PSM3_withErrCorr=readmatrix([DATA_ROOT_sTpsm,'\lc_T_psm_PSM3__withErrCorr.csv']);
data_PSM3_withoutErrCorr=readmatrix([DATA_ROOT_sTpsm,'\lc_T_psm_PSM3__withoutErrCorr.csv']);

%%%%%%%%%%PSM1 with Error Correction
%Trying different rotations about k Aruco because they weren't aligned
%perfectly
%angles_to_try=[-10:0.25:10]; %Angles in degrees
% for i=[1:length(angles_to_try)]
%     r_angle=angles_to_try(i);
%     [transDiff_PSM1_withErrCorr,angleDiff_PSM1_withErrCorr]=compareFramesAbsolute(data_PSM1_withErrCorr(:,5:16),data_PSM1_withErrCorr(:,18:29),r_angle);
% 
%     %Getting norm of translation error
%     trans_norm=vecnorm(transDiff_PSM1_withErrCorr,2,2);
%     mean_trans_norm=mean(trans_norm);
%     mean_angle_diff=mean(angleDiff_PSM1_withErrCorr);
% 
%     disp(['angle: ',num2str(r_angle)]);
%     disp(['mean_trans_norm: ',num2str(mean_trans_norm)]);
%     disp(['mean_angle_diff: ',num2str(mean_angle_diff)]);
% 
% 
% end

%best angle r_angle=-2.75
r_angle=-2.75;
[transDiff_PSM1_withErrCorr,angleDiff_PSM1_withErrCorr]=compareFramesAbsolute(data_PSM1_withErrCorr(:,5:16),data_PSM1_withErrCorr(:,18:29),r_angle);

%Analyzing Results
trans_norm_PSM1_withErrCorr=vecnorm(transDiff_PSM1_withErrCorr,2,2).*1000;
mean_trans_norm=mean(trans_norm_PSM1_withErrCorr);
std_trans_norm=std(trans_norm_PSM1_withErrCorr);

mean_angle_diff=mean(angleDiff_PSM1_withErrCorr);
std_angle_diff=std(angleDiff_PSM1_withErrCorr.*(180/pi));
disp('PSM1 With Err Correction')
disp(['mean_trans_norm: ',num2str(mean_trans_norm)]);
disp(['mean_angle_diff: ',num2str(mean_angle_diff)]);

%%%%%%%%%%PSM1 without Error Correction
[transDiff_PSM1_withoutErrCorr,angleDiff_PSM1_withoutErrCorr]=compareFramesAbsolute(data_PSM1_withoutErrCorr(:,5:16),data_PSM1_withoutErrCorr(:,18:29),r_angle);

%Analyzing Results
trans_norm_PSM1_withoutErrCorr=vecnorm(transDiff_PSM1_withoutErrCorr,2,2).*1000;
mean_trans_norm=mean(trans_norm_PSM1_withoutErrCorr);
std_trans_norm=std(trans_norm_PSM1_withoutErrCorr);
mean_angle_diff=mean(angleDiff_PSM1_withoutErrCorr);
std_angle_diff=std(angleDiff_PSM1_withoutErrCorr.*(180/pi));
disp('PSM1 Without Err Correction')
disp(['mean_trans_norm: ',num2str(mean_trans_norm)]);
disp(['mean_angle_diff: ',num2str(mean_angle_diff)]);


%%%%%%%%%%PSM3 with Error Correction
%Trying different rotations about k Aruco because they weren't aligned
%perfectly
% angles_to_try=[-10:0.25:10]; %Angles in degrees
% for i=[1:length(angles_to_try)]
%     r_angle=angles_to_try(i);
%     [transDiff_PSM3_withErrCorr,angleDiff_PSM3_withErrCorr]=compareFramesAbsolute(data_PSM3_withErrCorr(:,5:16),data_PSM3_withErrCorr(:,18:29),r_angle);
% 
%     %Getting norm of translation error
%     trans_norm=vecnorm(transDiff_PSM3_withErrCorr,2,2);
%     mean_trans_norm=mean(trans_norm);
%     mean_angle_diff=mean(angleDiff_PSM3_withErrCorr);
% 
%     disp(['angle: ',num2str(r_angle)]);
%     disp(['mean_trans_norm: ',num2str(mean_trans_norm)]);
%     disp(['mean_angle_diff: ',num2str(mean_angle_diff)]);
% 
% 
% end

%best angle r_angle=7
r_angle=7;
[transDiff_PSM3_withErrCorr,angleDiff_PSM3_withErrCorr]=compareFramesAbsolute(data_PSM3_withErrCorr(:,5:16),data_PSM3_withErrCorr(:,18:29),r_angle);

%Analyzing Results
trans_norm_PSM3_withErrCorr=vecnorm(transDiff_PSM3_withErrCorr,2,2).*1000;
mean_trans_norm=mean(trans_norm_PSM3_withErrCorr);
std_trans_norm=std(trans_norm_PSM3_withErrCorr);
mean_angle_diff=mean(angleDiff_PSM3_withErrCorr);
std_angle_diff=std(angleDiff_PSM3_withErrCorr.*(180/pi));
disp('PSM3 With Err Correction')
disp(['mean_trans_norm: ',num2str(mean_trans_norm)]);
disp(['mean_angle_diff: ',num2str(mean_angle_diff)]);


%%%%%%%%%%PSM3 without Error Correction
r_angle=-2.75;
[transDiff_PSM3_withoutErrCorr,angleDiff_PSM3_withoutErrCorr]=compareFramesAbsolute(data_PSM3_withoutErrCorr(:,5:16),data_PSM3_withoutErrCorr(:,18:29),r_angle);

%Analyzing Results
trans_norm_PSM3_withoutErrCorr=vecnorm(transDiff_PSM3_withoutErrCorr,2,2).*1000;
mean_trans_norm=mean(trans_norm_PSM3_withoutErrCorr);
std_trans_norm=std(trans_norm_PSM3_withoutErrCorr);
mean_angle_diff=mean(angleDiff_PSM3_withoutErrCorr);
std_angle_diff=std(angleDiff_PSM3_withoutErrCorr.*(180/pi));
disp('PSM3 Without Err Correction')
disp(['mean_trans_norm: ',num2str(mean_trans_norm)]);
disp(['mean_angle_diff: ',num2str(mean_angle_diff)]);



%%%%%%%Plotting Results (4x4 box plots)
figure;

subplot(1,2,1);
boxplot(trans_norm_PSM1_withErrCorr);
subtitle('Translation Error')
ylabel('Euc Norm (mm)')

subplot(1,2,2);
boxplot(angleDiff_PSM1_withErrCorr.*(180/pi));
subtitle('Rotation Error')
ylabel('Degrees')
sgtitle('PSM 1 With Error Correction')

figure;

subplot(1,2,1);
boxplot(trans_norm_PSM1_withoutErrCorr);
subtitle('Translation Error')
ylabel('Euc Norm (mm)')

subplot(1,2,2);
boxplot(angleDiff_PSM1_withoutErrCorr.*(180/pi));
subtitle('Rotation Error')
ylabel('Degrees')
sgtitle('PSM 1 Without Error Correction')


figure;

subplot(1,2,1);
boxplot(trans_norm_PSM3_withErrCorr);
subtitle('Translation Error')
ylabel('Euc Norm (mm)')

subplot(1,2,2);
boxplot(angleDiff_PSM3_withErrCorr.*(180/pi));
subtitle('Rotation Error')
ylabel('Degrees')
sgtitle('PSM 3 With Error Correction')


figure;

subplot(1,2,1);
boxplot(trans_norm_PSM3_withoutErrCorr);
subtitle('Translation Error')
ylabel('Euc Norm (mm)')

subplot(1,2,2);
boxplot(angleDiff_PSM3_withoutErrCorr.*(180/pi));
subtitle('Rotation Error')
ylabel('Degrees')
sgtitle('PSM 3 Without Error Correction')





%%%%%%%%%%%%%%%%%%%%%%%%%%%%Functions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [NDI_Angles,NDI_Translations,Frame_Angles,Frame_Translations]=compareFrameToNDI(NDI_Frames,Estim_Frames,NDI_T_Object,plot_frames_flag)
    %NDI_Frames & Estim Frames: rows = frames, columns = homogeneous indices
    %NDI_T_Object: Rigid transform between NDI tracker to aruco marker frame
    %Returns matri

    %All NDI Data is in mm, convert to meters to match ecm convention
    NDI_Frames(:,1:3)=NDI_Frames(:,1:3)./1000;
    
    [row,col]=size(NDI_Frames); %Gets the number of rows

    NDI_Angles=zeros([row-1,3]);
    Frame_Angles=zeros([row-1,3]); %Rotation Difference in euler angles (rotx,roty,rotz)

    NDI_Translations=zeros([row-1,3]);
    Frame_Translations=zeros([row-1,3]); %Translation where each column=x,y,z

    Estim_Row=Estim_Frames(1,:); %Extracts current row of estimated frame
    NDI_Row=NDI_Frames(1,:);   %Extracts current row of NDI frames

    %Convets the row to a frame
    Estim_Frame=convertCSVRowToHomo(Estim_Row);
    NDI_Frame=convertCSVRowToHomo(NDI_Row);

    Estim_Frame_Past=Estim_Frame;
    NDI_Frame_Past=NDI_Frame;

    for i=[2:row]   %Loops for the number of rows
        Estim_Row=Estim_Frames(i,:); %Extracts current row of estimated frame
        NDI_Row=NDI_Frames(i,:);   %Extracts current row of NDI frames

        %Convets the row to a frame
        Estim_Frame=convertCSVRowToHomo(Estim_Row);
        NDI_Frame=convertCSVRowToHomo(NDI_Row);

        % figure;
        % poseplot(Estim_Frame(1:3,1:3),Estim_Frame(1:3,4));
        % hold on
        % poseplot(NDI_Frame(1:3,1:3),NDI_Frame(1:3,4));
        % hold off
        % legend('Estim Frame','NDI Frame');

        %Right multiplies NDI frame by rigid tranform to get it to corner
        %of calibration object
        NDI_Frame=NDI_Frame*NDI_T_Object;
        % poseplot(NDI_Frame(1:3,1:3),NDI_Frame(1:3,4));
        % hold off
        % legend('Estim Frame','NDI Frame','NDI Frame Transformed');

        %%%%Computing transform between consecutive frames (relative)%%%
        
        %Estimated Frame:
        estimpast_T_estimcurr=invHomo(Estim_Frame_Past)*Estim_Frame; %Get frame motion
    
        %NDI Frame
        ndipast_T_ndicurr=invHomo(NDI_Frame_Past)*NDI_Frame; %Get frame motion
        %ndipast_T_ndicurr=ndipast_T_ndicurr;%*NDI_T_Object;
    
        % figure;
        % poseplot(Estim_Frame(1:3,1:3),Estim_Frame(1:3,4));
        % hold on
        % poseplot(NDI_Frame(1:3,1:3),NDI_Frame(1:3,4));
        % poseplot(Estim_Frame_Past(1:3,1:3),Estim_Frame_Past(1:3,4));
        % poseplot(NDI_Frame_Past(1:3,1:3),NDI_Frame_Past(1:3,4));
        % hold off
        % legend('Estim Frame Curr','NDI Frame Curr','Estim Frame Past','NDI Frame Past');
        % figure;
        % poseplot(estimpast_T_estimcurr(1:3,1:3),estimpast_T_estimcurr(1:3,4));
        % hold on
        % poseplot(ndipast_T_ndicurr(1:3,1:3),ndipast_T_ndicurr(1:3,4));
        % hold off
        % legend('Estim Frame Motion','NDI Frame Motion');
        
    
        %%%%Extracting translation and rotation angles%%%%
        
        %Estimated Frame
        Frame_Translations(i-1,:)=estimpast_T_estimcurr(1:3,4);
    
         
        %NDI Frames
        NDI_Translations(i-1,:)=ndipast_T_ndicurr(1:3,4);

        
        Estim_Frame_Past=Estim_Frame;
        NDI_Frame_Past=NDI_Frame;


    end


end


function [translation_diff,angle_diff]=compareFramesAbsolute(Aurco_Frames,Estim_Frames,r_angle)
    [row,col]=size(Aurco_Frames); %Gets the number of rows

    translation_diff=zeros([row,3]);
    angle_diff=zeros([row,1]); %Each row is rotation in degrees



    translation_x=0.631*0.0254; %Multiple by inches to meters conversion
    translation_y=-1.376*0.0254;
    translation_z=-0.231*0.0254;
    aruco_T_psm=[-1,0,0,translation_x;
                 0,0,1,translation_y;
                 0,1,0,translation_z;
                 0,0,0,1]; %Rigid transformation from aruco marker to psm grasper
    Trz=rotateZ(r_angle);

    for i=[1:row]   %Loops for the number of rows
        Estim_Raw=Estim_Frames(i,:); %Extracts current row of estimated frame
        Estim_Frame=convertRowToHomo(Estim_Raw); %Converts the data row to a homogeneous transform

        %Gets the frames
        Aruco_Raw=Aurco_Frames(i,:);
        Aruco_Frame=convertRowToHomo(Aruco_Raw);
        
        lc_T_psmEstim=Estim_Frame;
        lc_T_psmAc=Aruco_Frame*Trz*aruco_T_psm;
        
        %Gets translatoin difference
        translation_diff(i,:)=lc_T_psmEstim(1:3,4)-lc_T_psmAc(1:3,4);

        %Gets Aruco Rotation angle
        Ac_RAngle=rotm2axang(lc_T_psmAc(1:3,1:3));
        Ac_RAngle=Ac_RAngle(4);

        %Gets Kinematics Rotation Angle
        Estim_RAngle=rotm2axang(lc_T_psmEstim(1:3,1:3));
        Estim_RAngle=Estim_RAngle(4);

        angle_diff(i)=abs(Ac_RAngle-Estim_RAngle);
        
        %Plots the frame
        % figure;
        % poseplot(lc_T_psmEstim(1:3,1:3),lc_T_psmEstim(1:3,4));
        % hold on
        % poseplot(lc_T_psmAc(1:3,1:3),lc_T_psmAc(1:3,4));
        % legend("Estimated Pose","Actual Pose (AruCo Based)")
        % hold off

    end


end

function [TRz]=rotateZ(theta)
theta=theta*(pi/180);
TRz=[cos(theta),-sin(theta),0,0;
    sin(theta),cos(theta),0,0;
    0,0,1,0;
    0,0,0,1];

end




function [frame]=convertCSVRowToHomo(csv_row)
    frame=[csv_row(4),csv_row(5),csv_row(6),csv_row(1);
        csv_row(7),csv_row(8),csv_row(9),csv_row(2);
        csv_row(10),csv_row(11),csv_row(12),csv_row(3);
        0,0,0,1];
end

function T_inv = invHomo(T)
    % Extract the rotation part (3x3) and the translation part (3x1)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    
    % Compute the inverse rotation (which is the transpose of R)
    R_inv = inv(R);
    
    % Compute the inverse translation
    t_inv = -R_inv * t;
    
    % Construct the inverse transformation matrix
    T_inv = eye(4);
    T_inv(1:3, 1:3) = R_inv;
    T_inv(1:3, 4) = t_inv;
end