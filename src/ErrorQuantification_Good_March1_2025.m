clear
close all
clc


%%%%%%%%%%%%%%%%%%%%%%%%Cam-To-Scene Validation First%%%%%%%%%%%%%%%%%%%%%%
%% Reading in data
DATA_ROOT_CamToScene='..\resources\validation\camToScene\DecData';

%Gets data for multiple and single scene registration for left/right cams
data_multiple_left=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_multiple_left.csv']);
data_multiple_right=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_multiple_right.csv']);
data_single_left=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_single_left.csv']);
data_single_right=xlsread([DATA_ROOT_CamToScene,'\cam_to_scene_single_right.csv']);

%% Setting NDI_T_ARUCO Transform
NDI_T_ARUCO_RIGID_TRANSFORM=[0,0,-1,0.0745744;
                            1,0,0,-0.05207;
                            0,-1,0,-0.0485902;
                            0,0,0,1];

%% Computing Scene Registration errors and Stats

%%%%%%%%%Multiple Left (all vals in meters and degrees)
[NDI_mul_left_tanslation,estim_mul_left_translation,angle_err,NDI_mul_left_euler,...
    estim_mul_left_euler]=...
    compareFramesRelative(data_multiple_left(1,5:16),data_multiple_left(2:end,5:16),data_multiple_left(1,19:30),...
    data_multiple_left(2:end,19:30),NDI_T_ARUCO_RIGID_TRANSFORM);

NDI_mul_left_tanslation=NDI_mul_left_tanslation.*1000; %to mm
estim_mul_left_translation=estim_mul_left_translation.*1000; %to mm


[trans_err_abs_mul_left,trans_mean,trans_std]=translationStats(NDI_mul_left_tanslation,estim_mul_left_translation);
[angle_error_abs_mul_left,angle_mean,angle_std]=angleStats(angle_err);
disp(["*************"]);
disp(["Scene Registration Left"]);
disp(["Trans Error Mean: ",num2str(trans_mean)," STD=",num2str(trans_std)," (mm)"]);
disp(["|X| Error Mean: ",num2str(mean(trans_err_abs_mul_left(:,1)))," STD=",num2str(trans_std)," (mm)"]);
disp(["|Y| Error Mean: ",num2str(mean(trans_err_abs_mul_left(:,2)))," STD=",num2str(trans_std)," (mm)"]);
disp(["|Z| Error Mean: ",num2str(mean(trans_err_abs_mul_left(:,3)))," STD=",num2str(trans_std)," (mm)"]);
disp(["Angle Error Mean: ",num2str(angle_mean)," STD=",num2str(angle_std)," (deg)"]);


%%%%%%%%%Multiple Right (all vals in meters and degrees)
[NDI_mul_right_tanslation,estim_mul_right_translation,angle_err,NDI_mul_right_euler,...
    estim_mul_right_euler]=...
    compareFramesRelative(data_multiple_right(1,5:16),data_multiple_right(2:end,5:16),data_multiple_right(1,19:30),...
    data_multiple_right(2:end,19:30),NDI_T_ARUCO_RIGID_TRANSFORM);

NDI_mul_right_tanslation=NDI_mul_right_tanslation.*1000; %to mm
estim_mul_right_translation=estim_mul_right_translation.*1000; %to mm


[trans_err_abs_mul_right,trans_mean,trans_std]=translationStats(NDI_mul_right_tanslation,estim_mul_right_translation);
[angle_error_abs_mul_right,angle_mean,angle_std]=angleStats(angle_err);
disp(["*************"]);
disp(["Scene Registration Right"]);
disp(["Trans Error Mean: ",num2str(trans_mean)," STD=",num2str(trans_std)," (mm)"]);
disp(["|X| Error Mean: ",num2str(mean(trans_err_abs_mul_right(:,1)))," STD=",num2str(trans_std)," (mm)"]);
disp(["|Y| Error Mean: ",num2str(mean(trans_err_abs_mul_right(:,2)))," STD=",num2str(trans_std)," (mm)"]);
disp(["|Z| Error Mean: ",num2str(mean(trans_err_abs_mul_right(:,3)))," STD=",num2str(trans_std)," (mm)"]);
disp(["Angle Error Mean: ",num2str(angle_mean)," STD=",num2str(angle_std)," (deg)"]);


%% Plotting scene registration results
figuresaveroot='C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Code\dvFlexAR_PaperFigures\';
%%%%%%Left Multiple
plotTranslationsXYZ(estim_mul_left_translation,NDI_mul_left_tanslation,'Translation Left',[figuresaveroot,'translationplot_leftmultiple.svg']);%Display translations
plotEuler(estim_mul_left_euler,NDI_mul_left_euler,'Euler Angles Left',[figuresaveroot,'eulerplot_leftmultiple.svg']);

%%%%%%Right Multiple
plotTranslationsXYZ(estim_mul_right_translation,NDI_mul_right_tanslation,'Translation Right',[figuresaveroot,'translationplot_rightmultiple.svg']);%Display translations
plotEuler(estim_mul_right_euler,NDI_mul_right_euler,'Euler Angles Right',[figuresaveroot,'eulerplot_rightmultiple.svg']);

%%%%%Plotting Abs Translation Err Box Plots
plotAbsErrorOfAll([trans_err_abs_mul_left,sqrt(sum(trans_err_abs_mul_left.^2,2))],...
    [trans_err_abs_mul_right,sqrt(sum(trans_err_abs_mul_right.^2,2))],{"Left Cam","Right Cam"},...
    "Scene Registration Translation Error",[figuresaveroot,'boxplot_translationerror_multiple.svg']);

%%%%%Plotting Abs Angle Error Box Plots
plotAbsAngleErr(angle_error_abs_mul_left,angle_error_abs_mul_right,{"Left Cam","Right Cam"},...
    "Scene Registration Angle Error",[figuresaveroot,'boxplot_angleerror_multiple.svg'])
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%Functions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Compares motions of NDI and Vision Estimation to "base frame"
function [NDI_translations,estim_translations,angle_err,NDI_euler,estim_euler]=compareFramesRelative(NDI_base,NDI_frames,estim_base,estim_frames,NDI_T_Object)
    %NDI_base,estim_base: Row which is equivalent to homogeneous transform,
    %it is the relative "base" we compare motions to
    %NDI_Frames & Estim Frames: rows = frames, columns = homogeneous indices
    %NDI_T_Object: Rigid transform between NDI tracker to aruco marker frame
    %Returns matri

    %All NDI Data is in mm, convert to meters to match ecm convention
    NDI_frames(:,1:3)=NDI_frames(:,1:3)./1000;
    
    [row,col]=size(NDI_frames); %Gets the number of rows in the dataset

    %Arrays to store the angle error (two different ways to calculate based
    %on: http://www.boris-belousov.net/2016/12/01/quat-dist/ 
    %geodesic_angle_err=zeros([row,1]);
    angle_err=zeros([row,1]);

    %Arrays to store the translations
    NDI_translations=zeros([row,3]);
    estim_translations=zeros([row,3]); %Translation where each column=x,y,z

    %Arrays to store euler angles
    NDI_euler = zeros([row,3]);  % Stores Euler angles for NDI
    estim_euler = zeros([row,3]); % Stores Euler angles for estimation

    %Converts base frame rows to homogeneous transforms
    NDI_base_frame=convertCSVRowToHomo(NDI_base);
    NDI_base_frame(1:3,4)=NDI_base_frame(1:3,4)./1000;
    estim_base_frame=convertCSVRowToHomo(estim_base);



    for i=[1:row]   %Loops for the number of rows and stores the motions

        estim_row=estim_frames(i,:); %Extracts current row of estimated frame
        NDI_row=NDI_frames(i,:);   %Extracts current row of NDI frames

        %Convets the row to a homogeneous transform
        estim_frame=convertCSVRowToHomo(estim_row);
        NDI_frame=convertCSVRowToHomo(NDI_row);

        %Gets the motion of the estimated frame (relative to base frame)
        estimbase_T_estimcurr=invHomo(estim_base_frame)*estim_frame; 

        %Gets the motion of the NDI frame (relative to base frame)
        NDIbase_T_NDIcurr=invHomo(NDI_base_frame*NDI_T_Object)*NDI_frame*NDI_T_Object;      
    
        %%%%Extracting translation and rotation angles%%%%
        
        %Translations
        estim_translations(i,:)=estimbase_T_estimcurr(1:3,4); %Estimated
        NDI_translations(i,:)=NDIbase_T_NDIcurr(1:3,4); %Actual

        %Angle errors
        %geodesic_angle_err(i)=geodesicDistance(estimbase_T_estimcurr,NDIbase_T_NDIcurr);
        angle_err(i)=quaternionDistance(estimbase_T_estimcurr,NDIbase_T_NDIcurr);

        %Stores Euler Angles
        NDI_euler(i,:) = rad2deg(rotm2eul(NDIbase_T_NDIcurr(1:3,1:3), "XYZ"));
        estim_euler(i,:) = rad2deg(rotm2eul(estimbase_T_estimcurr(1:3,1:3), "XYZ"));

    end

end

function [frame]=convertCSVRowToHomo(csv_row)
    frame=[csv_row(4),csv_row(5),csv_row(6),csv_row(1);
        csv_row(7),csv_row(8),csv_row(9),csv_row(2);
        csv_row(10),csv_row(11),csv_row(12),csv_row(3);
        0,0,0,1];
end

%Computes inverse of homogeneous transform
function T_inv = invHomo(T)
    % Extract the rotation part (3x3) and the translation part (3x1)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    
    % Compute the inverse rotation (which is the transpose of R)
    R_inv = R';
    
    % Compute the inverse translation
    t_inv = -R_inv * t;
    
    % Construct the inverse transformation matrix
    T_inv = eye(4);
    T_inv(1:3, 1:3) = R_inv;
    T_inv(1:3, 4) = t_inv;
end

function rotationError=geodesicDistance(T1,T2)
R_error=T1(1:3,1:3)*(T2(1:3,1:3)'); %Rotation between the two rotations
rotationError=acos((trace(R_error)-1)/2);
rotationError=rad2deg(rotationError);
end

function rotationError=quaternionDistance(T1,T2)
q1 = rotm2quat(T1(1:3,1:3));
q2 = rotm2quat(T2(1:3,1:3));
q_error=quatmultiply(q2,quatinv(q1));
rotationError=2*acos(q_error(1));
rotationError=rad2deg(rotationError);
end


%%%%%%%%%%Stats Functions

function [trans_err_abs,trans_mean,trans_std]=translationStats(translation1,translation2)
    trans_err=(translation1-translation2); %In mm
    trans_err_L2=sqrt(sum(trans_err.^2,2));
    trans_mean=mean(trans_err_L2);
    trans_std=std(trans_err_L2);
    trans_err_abs=abs(trans_err);
end

function [angle_error_abs,angle_mean,angle_std]=angleStats(angle_error)
    angle_error_abs=abs(angle_error);
    angle_mean=mean(abs(angle_error_abs));
    angle_std=std(angle_error_abs);   
end


%%%%%%%%%%Plotting Functions
function plotTranslationsXYZ(translation1,translation2,plotoveralltitle,savefile)
    % translation1 & translation2 have columns x,y,z
    [row,~] = size(translation1);
    x_axis = 1:row;

    % Define shades of gray
    dark_gray = [0.3, 0.3, 0.3];
    light_gray = [0.6, 0.6, 0.6];

    % Create figure and set background to white
    figure;
    set(gcf, 'Color', 'w');

    % X-axis translation
    subplot(3,1,1); 
    p1 = plot(x_axis, translation2(:,1), '.-', 'Color', dark_gray, 'LineWidth', 1.9, 'MarkerSize', 9); hold on;
    p2 = plot(x_axis, translation1(:,1), '.--', 'Color', light_gray, 'LineWidth', 1.9, 'MarkerSize', 9);
    set(gca, 'FontSize', 12);
    xlim([1, row]);    
    ylabel('X (mm)', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Y-axis translation
    subplot(3,1,2); 
    plot(x_axis, translation2(:,2), '.-', 'Color', dark_gray, 'LineWidth', 1.9, 'MarkerSize', 9); hold on;
    plot(x_axis, translation1(:,2), '.--', 'Color', light_gray, 'LineWidth', 1.9, 'MarkerSize', 9);
    set(gca, 'FontSize', 12);
    xlim([1, row]);
    ylabel('Y (mm)', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Z-axis translation
    ax3 = subplot(3,1,3); % Attach legend to this subplot
    plot(x_axis, translation2(:,3), '.-', 'Color', dark_gray, 'LineWidth', 1.9, 'MarkerSize', 9); hold on;
    plot(x_axis, translation1(:,3), '.--', 'Color', light_gray, 'LineWidth', 1.9, 'MarkerSize', 9);
    set(gca, 'FontSize', 12);
    xlim([1, row]);
    ylabel('Z (mm)', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');
    xlabel('Sample', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Title
    sgtitle(plotoveralltitle, 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Set figure size
    set(gcf, 'Position', [100, 100, 550, 700]);  % (left, bottom, width, height in pixels)

    % Save figure
    saveas(gcf, savefile);
end

function plotEuler(euler1,euler2,plotoveralltitle,savefile)
    % translation1 & translation2 have columns x,y,z
    [row,~] = size(euler1);
    x_axis = 1:row;

    % Define shades of gray
    dark_gray = [0.3, 0.3, 0.3];
    light_gray = [0.6, 0.6, 0.6];

    % Create figure and set background to white
    figure;
    set(gcf, 'Color', 'w');

    % X-axis translation
    subplot(3,1,1); 
    p1 = plot(x_axis, euler2(:,1), '.-', 'Color', dark_gray, 'LineWidth', 1.9, 'MarkerSize', 9); hold on;
    p2 = plot(x_axis, euler1(:,1), '.--', 'Color', light_gray, 'LineWidth', 1.9, 'MarkerSize', 9);
    set(gca, 'FontSize', 12);
    xlim([1, row]);    
    ylabel('Roll (deg)', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Y-axis translation
    subplot(3,1,2); 
    plot(x_axis, euler2(:,2), '.-', 'Color', dark_gray, 'LineWidth', 1.9, 'MarkerSize', 9); hold on;
    plot(x_axis, euler1(:,2), '.--', 'Color', light_gray, 'LineWidth', 1.9, 'MarkerSize', 9);
    set(gca, 'FontSize', 12);
    xlim([1, row]);
    ylabel('Pitch (deg)', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Z-axis translation
    ax3 = subplot(3,1,3); % Attach legend to this subplot
    plot(x_axis, euler2(:,3), '.-', 'Color', dark_gray, 'LineWidth', 1.9, 'MarkerSize', 9); hold on;
    plot(x_axis, euler1(:,3), '.--', 'Color', light_gray, 'LineWidth', 1.9, 'MarkerSize', 9);
    set(gca, 'FontSize', 12);
    xlim([1, row]);
    ylabel('Yaw (deg)', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');
    xlabel('Sample', 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Title
    sgtitle(plotoveralltitle, 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

    % Set figure size
    set(gcf, 'Position', [100, 100, 550, 700]);  % (left, bottom, width, height in pixels)

    % Save figure
    saveas(gcf, savefile);
end


function plotAbsErrorOfAll(leftMul_err,rightMul_err,grouplabels,figuretitle,savefile)

data=[leftMul_err,rightMul_err];
positions = [0.25 0.5 0.75 1 1.5 1.75 2 2.25]; %Space (no 5)

%Different shades of grey
gray4 = [0.2, 0.2, 0.2];
gray3 = [0.4, 0.4, 0.4];
gray2 = [0.6, 0.6, 0.6];
gray1 = [0.8, 0.8, 0.8];
color=[gray1;gray2;gray3;gray4;gray1;gray2;gray3;gray4];

figure;
set(gcf, 'Color', 'w');
boxplot(data,'Positions',positions,'Colors',color,'BoxStyle','filled','MedianStyle','target','Symbol','r+');%'BoxStyle','filled','MedianStyle','target','Symbol','r+');

%Formatting
xticks([mean(positions(1:4)),mean(positions(5:end))]);
set(gca, 'FontSize', 12);
xticklabels(grouplabels);
ax=gca;
ax.XAxis.FontSize=18;
ax.XAxis.FontName='Times';
ax.XAxis.FontWeight='bold';
ylabel('|Translation Error| (mm)','FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');
title(figuretitle, 'FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');

%h = findobj(gca, 'Tag', 'Box');
%h = flipud(h); % Flip to match ordering in the plot

% Apply colors
% for j = 1:length(h)
%     boxX = get(h(j), 'XData');  
%     boxY = get(h(j), 'YData'); 
%     patch(boxX, boxY, color(j, :), 'FaceAlpha', 0.5, 'EdgeColor', 'k');
% end
% 
%c = get(gca, 'Children');
%legend(c(1:4), '|X|', '|Y|','|Z|','L2 Norm');
set(gcf, 'Position', [100, 100, 450, 475]);  % (left, bottom, width, height in pixels)

hold on;
hLegend = gobjects(4,1); % Preallocate legend handles

legend_labels = {'|X|', '|Y|', '|Z|', 'L2 Norm'}; % Legend names

for j = 1:4 % Only 4 different colors, corresponding to each variable
    hLegend(j) = plot(nan, nan, 's', 'MarkerFaceColor', color(j,:), 'MarkerEdgeColor', 'k');
end

legend(hLegend, legend_labels, 'Location', 'best');

hold off;


saveas(gcf, savefile);
end

function plotAbsAngleErr(left_angle_error,right_angle_error,grouplabels,figuretitle,savefile)

data=[left_angle_error,right_angle_error];
positions = [0.25 0.75]; %Space (no 5)

%Different shades of grey
dark_gray = [0.3, 0.3, 0.3];
light_gray = [0.6, 0.6, 0.6];
color=[light_gray;dark_gray];

figure;
set(gcf, 'Color', 'w');
boxplot(data,'Positions',positions,'Colors',color,'BoxStyle','filled','MedianStyle','target','Symbol','r+');%'BoxStyle','filled','MedianStyle','target','Symbol','r+');

%Formatting
xticks(positions);
set(gca, 'FontSize', 12);
xticklabels(grouplabels);
ax=gca;
ax.XAxis.FontSize=18;
ax.XAxis.FontName='Times';
ax.XAxis.FontWeight='bold';
ylabel('|Angle Error| (deg)','FontSize', 20, 'FontName', 'Times', 'FontWeight', 'bold');
%title(figuretitle, 'FontSize', 20, 'FontName', 'Times', 'FontWeight',
%'bold'); =>no title for now

set(gcf, 'Position', [100, 100, 200, 300]);  % (left, bottom, width, height in pixels)

saveas(gcf, savefile);
end
