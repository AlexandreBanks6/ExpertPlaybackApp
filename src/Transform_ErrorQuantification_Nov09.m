clear
clc
close all

DATA_ROOT_CamToScene='..\resources\validation\camToScene';
DATA_ROOT_sTpsm='..\resources\validation\PSMPose';

% NDI_T_ARUCO_RIGID_TRANSFORM=[1,0,0,-0.05225;
%                             0,0,-1,0.076327;
%                             0,1,0,0.048387;
%                             0,0,0,1];

% NDI_T_ARUCO_RIGID_TRANSFORM=[0,0,1,-0.076327;
%                             1,0,0,-0.05225;
%                             0,1,0,0.048387;
%                             0,0,0,1];

% NDI_T_ARUCO_RIGID_TRANSFORM=[1,0,0,-0.05225;
%                             0,-1,0,-0.048387;
%                             0,0,-1,0.076327;
%                             0,0,0,1];

NDI_T_ARUCO_RIGID_TRANSFORM=[0,0,-1,0.0745744;
                            1,0,0,-0.05207;
                            0,-1,0,-0.0485902;
                            0,0,0,1];

% PSM_T_ARUCO_RIGID_TRANSFORM=[-1,0,0,0.0061722;
%                             0,0,1,0.0104394;
%                             0,1,0,0.0315722;
%                             0,0,0,1];

PSM_T_ARUCO_RIGID_TRANSFORM=[-1,0,0,0.004572;
                            0,0,1,0.0096774;
                            0,1,0,0.0325882;
                            0,0,0,1];

PSM3_T_ARUCO_RIGID_TRANSFORM=[1,0,0,-0.004572;
                            0,0,-1,-0.0096774;
                            0,1,0,0.0325882;
                            0,0,0,1];

%%%%%%%%%%%%%%%%%%%%%%%Cam-To-Scene Transforms%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Reading in data
data_multiple_left=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_multiple_left.csv']);
data_multiple_right=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_multiple_right.csv']);
data_single_left=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_single_left.csv']);
data_single_right=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_single_right.csv']);


% Analyzing Error Multiple Left 
[NDI_Mul_left_Angles,NDI_Mul_left_Tanslation,Estim_Mul_Left_Angles,Estim_Mul_Left_Translation]=...
    compareFramesRelative(data_multiple_left(:,5:16),data_multiple_left(:,19:30),...
    NDI_T_ARUCO_RIGID_TRANSFORM,0,"Left camTscene Multi-Frame Registration ");
%Get rid of first row (outlier)
NDI_Mul_left_Angles(1,:)=[];
NDI_Mul_left_Tanslation(1,:)=[];
Estim_Mul_Left_Angles(1,:)=[];
Estim_Mul_Left_Translation(1,:)=[];

[Mul_Left_TransError,Mul_Left_TransMean,Mul_Left_TransStd,Mul_Left_AngleError,...
    Mul_Left_AngleMean,Mul_Left_AngleStd]=runStats(NDI_Mul_left_Tanslation, ...
    Estim_Mul_Left_Translation,NDI_Mul_left_Angles,Estim_Mul_Left_Angles);
%Plotting Error
showErrorBoxPlots(abs(Mul_Left_TransError),abs(Mul_Left_AngleError),'Left camTscene Multi-Frame Registration ');




%%%%%%%%%%%%%%%%%%%%%lc_T_psm Transforms%%%%%%%%%%%%%%%%%%%%%%%%
%% Reading in data
data_PSM1_withErrCorr=readmatrix([DATA_ROOT_sTpsm,'\lc_T_psm_PSM1__withErrCorr.csv']);
data_PSM3_withErrCorr=readmatrix([DATA_ROOT_sTpsm,'\lc_T_psm_PSM3__withErrCorr.csv']);


data_PSM1_withErrCorr(6,:)=[];
data_PSM1_withErrCorr(6,:)=[];
data_PSM1_withErrCorr(11,:)=[];
data_PSM1_withErrCorr(12,:)=[];
data_PSM1_withErrCorr(13,:)=[];


%Analyzing PSM1 with Error Correction
[Aruco_Angles,Aruco_Tanslation,PSM1_Angles,PSM1_Translation]=...
    compareFramesAbsolute(data_PSM1_withErrCorr(:,5:16),data_PSM1_withErrCorr(:,18:29),...
    PSM_T_ARUCO_RIGID_TRANSFORM,0,"PSM1 camTpsm Tracking ");

[PSM1_TransError,PSM1_TransMean,PSM1_TransStd,PSM1_AngleError,...
    PSM1_AngleMean,PSM1_AngleStd]=runStatsAbsolute(Aruco_Tanslation, ...
    PSM1_Translation,Aruco_Angles,PSM1_Angles);

showErrorBoxPlots(abs(PSM1_TransError),abs(PSM1_AngleError),'PSM1 camTpsm Tracking ');

%[translation_diff,angle_diff]=compareFramesAbsoluteOld(data_PSM1_withErrCorr(:,5:16),data_PSM1_withErrCorr(:,18:29),PSM_T_ARUCO_RIGID_TRANSFORM);



%Analyzing PSM3 with Error Correction

data_PSM3_withErrCorr(2,:)=[];
data_PSM3_withErrCorr(3,:)=[];
data_PSM3_withErrCorr(3,:)=[];
data_PSM3_withErrCorr(5,:)=[];
data_PSM3_withErrCorr(6,:)=[];
data_PSM3_withErrCorr(10,:)=[];
data_PSM3_withErrCorr(10,:)=[];
data_PSM3_withErrCorr(16,:)=[];

[Aruco_Angles_psm3,Aruco_Tanslation_psm3,PSM3_Angles,PSM3_Translation]=...
    compareFramesAbsolute(data_PSM3_withErrCorr(:,5:16),data_PSM3_withErrCorr(:,18:29),...
    PSM3_T_ARUCO_RIGID_TRANSFORM,0,"PSM3 camTpsm Tracking");

[PSM3_TransError,PSM3_TransMean,PSM3_TransStd,PSM3_AngleError,...
    PSM3_AngleMean,PSM3_AngleStd]=runStatsAbsolute(Aruco_Tanslation_psm3, ...
    PSM3_Translation,Aruco_Angles_psm3,PSM3_Angles);

showErrorBoxPlots(abs(PSM3_TransError),abs(PSM3_AngleError),'PSM3 camTpsm Tracking');





%%%%%%%%%%%%%%%%%%%%%%%%%%%%Functions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [NDI_Angles,NDI_Translations,Frame_Angles,Frame_Translations]=compareFramesRelative(NDI_Frames,Estim_Frames,NDI_T_Object,plot_frames_flag,plot_name)
    %NDI_Frames & Estim Frames: rows = frames, columns = homogeneous indices
    %NDI_T_Object: Rigid transform between NDI tracker to aruco marker frame
    %Returns matri

    %All NDI Data is in mm, convert to meters to match ecm convention
    NDI_Frames(:,1:3)=NDI_Frames(:,1:3)./1000;
    Estim_Frames(:,1:3)=Estim_Frames(:,1:3);
    
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
    if(plot_frames_flag)       
        figure;
        hold on
    end

    for i=[2:row]   %Loops for the number of rows
        Estim_Row=Estim_Frames(i,:); %Extracts current row of estimated frame
        NDI_Row=NDI_Frames(i,:);   %Extracts current row of NDI frames

        %Convets the row to a frame
        Estim_Frame=convertCSVRowToHomo(Estim_Row);
        NDI_Frame=convertCSVRowToHomo(NDI_Row);

        %Right multiplies NDI frame by rigid tranform to get it to corner
        %of calibration object
        NDI_Frame=NDI_Frame*NDI_T_Object;

        %%%%Computing transform between consecutive frames (relative)%%%
        
        %Estimated Frame:
        estimpast_T_estimcurr=invHomo(Estim_Frame_Past)*Estim_Frame; %Get frame motion
    
        %NDI Frame
        ndipast_T_ndicurr=invHomo(NDI_Frame_Past)*NDI_Frame; %Get frame motion
        %ndipast_T_ndicurr=ndipast_T_ndicurr;%*NDI_T_Object;
    
        if(plot_frames_flag)

            poseplot(estimpast_T_estimcurr(1:3,1:3),estimpast_T_estimcurr(1:3,4).*100);
            poseplot(ndipast_T_ndicurr(1:3,1:3),ndipast_T_ndicurr(1:3,4).*100);
        end
        
    
        %%%%Extracting translation and rotation angles%%%%
        
        %Estimated Frame
        Frame_Translations(i-1,:)=estimpast_T_estimcurr(1:3,4);    
         
        %NDI Frames
        NDI_Translations(i-1,:)=ndipast_T_ndicurr(1:3,4);

        %Angles
        NDI_Angles(i-1,:)=rotm2eul(ndipast_T_ndicurr(1:3,1:3),"XYZ");
        Frame_Angles(i-1,:)=rotm2eul(estimpast_T_estimcurr(1:3,1:3),"XYZ");
        

        %%%%%%Updating Past
        Estim_Frame_Past=Estim_Frame;
        NDI_Frame_Past=NDI_Frame;


    end
    if(plot_frames_flag)
            plot3(Frame_Translations(:,1).*100,Frame_Translations(:,2).*100,Frame_Translations(:,3).*100);
            plot3(NDI_Translations(:,1).*100,NDI_Translations(:,2).*100,NDI_Translations(:,3).*100);
            axis equal;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title(plot_name);
            hold off
            legend('camTscene Frame Motion','NDI Frame Motion');            
    end


end

function [Aruco_Angles,Aruco_Translations,PSM_Angles,PSM_Translations]=compareFramesAbsolute(Aurco_Frames,PSM_Frames,PSM_T_Aruco,plot_frames_flag,plot_name)
    %NDI_Frames & Estim Frames: rows = frames, columns = homogeneous indices
    %NDI_T_Object: Rigid transform between NDI tracker to aruco marker frame
    %Returns matri

    
    [row,col]=size(Aurco_Frames); %Gets the number of rows

    Aruco_Angles=zeros([row,3]);
    PSM_Angles=zeros([row,3]); %Rotation Difference in euler angles (rotx,roty,rotz)

    Aruco_Translations=zeros([row,3]);
    PSM_Translations=zeros([row,3]); %Translation where each column=x,y,z

    if(plot_frames_flag)       
        figure;
        hold on
    end

    for i=[1:row]   %Loops for the number of rows
        PSM_Row=PSM_Frames(i,:); %Extracts current row of estimated frame
        Aruco_Row=Aurco_Frames(i,:);   %Extracts current row of NDI frames

        %Convets the row to a frame
        PSM_Frame=convertCSVRowToHomo(PSM_Row);
        Aruco_Frame=convertCSVRowToHomo(Aruco_Row);

        %Right multiplies PSM frame by rigid tranform to get it to corner
        %of aruco
        PSM_Frame=PSM_Frame*PSM_T_Aruco;

        %Plotting If needed
    
        if(plot_frames_flag)

            poseplot(PSM_Frame(1:3,1:3),PSM_Frame(1:3,4).*1000);
            poseplot(Aruco_Frame(1:3,1:3),Aruco_Frame(1:3,4).*1000);
        end
        
    
        %%%%Extracting translation and rotation angles%%%%
        
        %Estimated Frame
        PSM_Translations(i,:)=PSM_Frame(1:3,4);    
         
        %Aruco Frames
        Aruco_Translations(i,:)=Aruco_Frame(1:3,4);

        %Angle
        % PSM_Angle=rotm2axang(PSM_Frame(1:3,1:3));
        % PSM_Angle=PSM_Angle(4);
        % 
        % Ac_RAngle=rotm2axang(Aruco_Frame(1:3,1:3));
        % Ac_RAngle=Ac_RAngle(4);
        % 
        % Aruco_Angles(i)=Ac_RAngle;
        % PSM_Angles(i)=PSM_Angle;
        Aruco_Angles(i,:)=rotm2eul(Aruco_Frame(1:3,1:3),"XYZ");
        PSM_Angles(i,:)=rotm2eul(PSM_Frame(1:3,1:3),"XYZ");


        




    end
    if(plot_frames_flag)
            plot3(PSM_Translations(:,1).*1000,PSM_Translations(:,2).*1000,PSM_Translations(:,3).*1000);
            plot3(Aruco_Translations(:,1).*1000,Aruco_Translations(:,2).*1000,Aruco_Translations(:,3).*1000);
            axis equal;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title(plot_name);
            hold off
            legend('camera_T_psm','camera_T_aruco');            
    end


end



function [trans_err,mean_trans,std_trans,angle_err,mean_angle,std_angle]=runStats(translation1,translation2,angle1,angle2)
    trans_err=(translation1-translation2).*100; %In mm
    trans_err_L2=sqrt(sum(trans_err.^2,2));
    mean_trans=mean(trans_err_L2);
    std_trans=std(trans_err_L2);
    
    angle_err=(angle1-angle2).*(180/pi);  %In degrees
    angle_err_L2=sqrt(sum(angle_err.^2,2));
    mean_angle=mean(angle_err_L2);
    std_angle=std(angle_err_L2);

end

function [trans_err,mean_trans,std_trans,angle_err,mean_angle,std_angle]=runStatsAbsolute(translation1,translation2,angle1,angle2)
    trans_err=(translation1-translation2).*1000; %In mm
    trans_err_L2=sqrt(sum(trans_err.^2,2));
    mean_trans=mean(trans_err_L2);
    std_trans=std(trans_err_L2);
    
    angle_err=(angle1-angle2).*(180/pi);  %In degrees
    angle_err_L2=sqrt(sum(angle_err.^2,2));
    mean_angle=mean(angle_err_L2);
    std_angle=std(angle_err_L2);

end


function showErrorBoxPlots(trans_error,angle_error,compare_name)

    figure;
    h=boxplot(trans_error,'Labels',{'|x|' ,'|y|' ,'|z|' },'MedianStyle','line','Colors','rgb','BoxStyle','outline');
    ylabel('Absolute Error (mm)','FontName', 'Arial', 'FontSize', 14);
    xlabel('Axis','FontName', 'Arial', 'FontSize', 14);
    title([compare_name,'Position Error'],'FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
    set(h, 'LineWidth', 2); % Adjust the value as needed (e.g., 2 for thicker lines)

    figure;
    h=boxplot(angle_error,'Labels',{'|roll|' ,'|pitch|' ,'|yaw|' },'MedianStyle','line','Colors','rgb','BoxStyle','outline');
    ylabel('Absolute Error (degrees)','FontName', 'Arial', 'FontSize', 14);
    xlabel('Axis','FontName', 'Arial', 'FontSize', 14);
    title([compare_name,'Angle Error'],'FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
    set(h, 'LineWidth', 2); % Adjust the value as needed (e.g., 2 for thicker lines)
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