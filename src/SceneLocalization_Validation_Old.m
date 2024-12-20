<<<<<<< HEAD
clear
clc
close all

%% Reading in data and pre-processing

DataMat=xlsread('C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Data\ValidationForAlgo_March2024\validation_19-03-2024_18-01-58_.csv');
GuideMat=xlsread('C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Data\ValidationForAlgo_March2024\SceneRegistration_Modified.xlsx');


%Averaging quaternions from NDI Tracker
Trial_Numbers=DataMat(:,1);

N=max(Trial_Numbers);
New_Data=[]; %New Data which we find from averaging NDI Quaternions
for i=1:N
    %Loop for trials and take average of NDI quaternion
    Trial_Inds=Trial_Numbers==i;
    NDI_Translation=DataMat(Trial_Inds,3:5);
    Mean_Translation=mean(NDI_Translation,1);

    NDI_Quaternion=DataMat(Trial_Inds,6:9);
    NDI_Quaternion=quaternion(NDI_Quaternion);
    NDI_Quaternion=normalize(NDI_Quaternion);
    Mean_Quaternion=compact(meanrot(NDI_Quaternion));

    Indexes=find(Trial_Inds==1);
    Last_Index=Indexes(end);
    New_Data=[New_Data;[i,Mean_Translation,Mean_Quaternion,DataMat(Last_Index,12:18)]];    

end

%We get rid of trial 27 because it was a mistake
Del_Row=find(New_Data(:,1)==27);
New_Data(Del_Row,:)=[];

Del_Row=find(GuideMat(:,5)==27);
GuideMat(Del_Row,:)=[];



%% Aligning Recorded Notes and Captured Data, Finding Error

TrialNumbers_Notes=GuideMat(:,5);
[TrialNumbers_Notes,I]=sort(TrialNumbers_Notes);
GuideMat=GuideMat(I,:);
%TrialNumbers_Notes=GuideMat(:,5);

BaseMeasurements=[8,15,21,28,35,39,42,45,48]; %Array Containing the base measurements, 1 is inherently the first base

%Matrices used in calculating RMSE

rotation_error_NDI=[];
rotation_error_vision=[];

translation_error_NDI=[];
translation_error_vision=[];

%Get Rid of NaN trial numbers (these are ones that we do free-movement)
trial_numbers=GuideMat(:,5);
nans=isnan(trial_numbers);
GuideMat(nans,:)=[];



[rows,cols]=size(GuideMat);


base_pose_NDI=New_Data(1,2:8);
base_pose_vision=New_Data(1,9:15);

trial_numbers_guidemat=GuideMat(:,5);
trial_numbers_datamat=New_Data(:,1);

Translation_Counter=1;

for i=[2:rows]
    if any(trial_numbers_guidemat(i)==BaseMeasurements)
        %Update the base measurements and continue
        base_pose_NDI=New_Data(i,2:8);
        base_pose_vision=New_Data(i,9:15);
    else
        %Not the start of a new trial so use last base measurement
        indx=find(trial_numbers_datamat==trial_numbers_guidemat(i));
        pose_NDI=New_Data(indx,2:8);
        pose_vision=New_Data(indx,9:15);

        %Check if we are looking for translation or rotation error
    
        if ~isnan(GuideMat(i,2))    %We are looking at translation error
            if any(Translation_Counter==[10,11,12,28]) %Remove outliers from jumpiness
                Translation_Counter=Translation_Counter+1;
            else
                %Extract data rows:

                %Gold Standard Translation (manual)
                const=GuideMat(i,2); %Constant multiplier, 1:1mm
                trans_actual=const*1; %In mm
    
                %Amount of translation from vision
                translation_vision=pose_vision(1:3)-base_pose_vision(1:3);
                translation_vision=sqrt(sum(translation_vision.^2)); %Take euclidean distance of translation
                translation_vision=translation_vision*1000; %Convert to mm
           
                %Amount of translation from NDI
                translation_NDI=pose_NDI(1:3)-base_pose_NDI(1:3);
                translation_NDI=sqrt(sum(translation_NDI.^2));
                translation_NDI=translation_NDI*1000; %Convert to mm
    
    
                %Computing errors
                translation_error_vision=[translation_error_vision,translation_vision-trans_actual];
                translation_error_NDI=[translation_error_NDI,translation_NDI-trans_actual];
    
    
    
                % disp("Translation Vision: ");
                % disp(translation_vision);
                % disp("Translation NDI: ");
                % disp(translation_NDI);
                Translation_Counter=Translation_Counter+1;

            end
            
        
    
        else %Rotation error

            %Amount of translation from vision
            rotation_vision=rad2deg(dist(quaternion(pose_vision(4:7)),quaternion(base_pose_vision(4:7))));

            %Amount of translation from NDI
            rotation_NDI=rad2deg(dist(quaternion(pose_NDI(4:7)),quaternion(base_pose_NDI(4:7))));

            % disp("Rotation Vision: ");
            % disp(rotation_vision);
            % disp("Rotation NDI: ");
            % disp(rotation_NDI);
    
    
        end

        %Update the base measurement for next loop around, we are always
        %comparing to last motion
        base_pose_NDI=pose_NDI;
        base_pose_vision=pose_vision;

    end

end



%% Plotting Translation errors

%Post process translation errors
translation_error_vision=abs(translation_error_vision)';

translation_error_NDI=abs(translation_error_NDI)';
const_NDI=mean(translation_error_NDI);
translation_error_NDI=abs(translation_error_NDI-const_NDI);

figure(1);
box_data=[translation_error_vision,translation_error_NDI];

%Grouping variable for x-axis
g1={'Vision Algorithm','NDI Tracker'};
boxplot(box_data,g1,'BoxStyle','filled','MedianStyle','target');
ylabel('Translation Error (mm)','FontWeight','bold','FontSize',12);
title('Comparison of Vision-Based and NDI Translation Tracking','FontSize',14,'FontWeight','bold')





=======
clear
clc
close all

%% Reading in data and pre-processing

DataMat=xlsread('C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Data\ValidationForAlgo_March2024\validation_19-03-2024_18-01-58_.csv');
GuideMat=xlsread('C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Data\ValidationForAlgo_March2024\SceneRegistration_Modified.xlsx');


%Averaging quaternions from NDI Tracker
Trial_Numbers=DataMat(:,1);

N=max(Trial_Numbers);
New_Data=[]; %New Data which we find from averaging NDI Quaternions
for i=1:N
    %Loop for trials and take average of NDI quaternion
    Trial_Inds=Trial_Numbers==i;
    NDI_Translation=DataMat(Trial_Inds,3:5);
    Mean_Translation=mean(NDI_Translation,1);

    NDI_Quaternion=DataMat(Trial_Inds,6:9);
    NDI_Quaternion=quaternion(NDI_Quaternion);
    NDI_Quaternion=normalize(NDI_Quaternion);
    Mean_Quaternion=compact(meanrot(NDI_Quaternion));

    Indexes=find(Trial_Inds==1);
    Last_Index=Indexes(end);
    New_Data=[New_Data;[i,Mean_Translation,Mean_Quaternion,DataMat(Last_Index,12:18)]];    

end

%We get rid of trial 27 because it was a mistake
Del_Row=find(New_Data(:,1)==27);
New_Data(Del_Row,:)=[];

Del_Row=find(GuideMat(:,5)==27);
GuideMat(Del_Row,:)=[];



%% Aligning Recorded Notes and Captured Data, Finding Error

TrialNumbers_Notes=GuideMat(:,5);
[TrialNumbers_Notes,I]=sort(TrialNumbers_Notes);
GuideMat=GuideMat(I,:);
%TrialNumbers_Notes=GuideMat(:,5);

BaseMeasurements=[8,15,21,28,35,39,42,45,48]; %Array Containing the base measurements, 1 is inherently the first base

%Matrices used in calculating RMSE

rotation_error_NDI=[];
rotation_error_vision=[];

translation_error_NDI=[];
translation_error_vision=[];

%Get Rid of NaN trial numbers (these are ones that we do free-movement)
trial_numbers=GuideMat(:,5);
nans=isnan(trial_numbers);
GuideMat(nans,:)=[];



[rows,cols]=size(GuideMat);


base_pose_NDI=New_Data(1,2:8);
base_pose_vision=New_Data(1,9:15);

trial_numbers_guidemat=GuideMat(:,5);
trial_numbers_datamat=New_Data(:,1);

Translation_Counter=1;

for i=[2:rows]
    if any(trial_numbers_guidemat(i)==BaseMeasurements)
        %Update the base measurements and continue
        base_pose_NDI=New_Data(i,2:8);
        base_pose_vision=New_Data(i,9:15);
    else
        %Not the start of a new trial so use last base measurement
        indx=find(trial_numbers_datamat==trial_numbers_guidemat(i));
        pose_NDI=New_Data(indx,2:8);
        pose_vision=New_Data(indx,9:15);

        %Check if we are looking for translation or rotation error
    
        if ~isnan(GuideMat(i,2))    %We are looking at translation error
            if any(Translation_Counter==[10,11,12,28]) %Remove outliers from jumpiness
                Translation_Counter=Translation_Counter+1;
            else
                %Extract data rows:

                %Gold Standard Translation (manual)
                const=GuideMat(i,2); %Constant multiplier, 1:1mm
                trans_actual=const*1; %In mm
    
                %Amount of translation from vision
                translation_vision=pose_vision(1:3)-base_pose_vision(1:3);
                translation_vision=sqrt(sum(translation_vision.^2)); %Take euclidean distance of translation
                translation_vision=translation_vision*1000; %Convert to mm
           
                %Amount of translation from NDI
                translation_NDI=pose_NDI(1:3)-base_pose_NDI(1:3);
                translation_NDI=sqrt(sum(translation_NDI.^2));
                translation_NDI=translation_NDI*1000; %Convert to mm
    
    
                %Computing errors
                translation_error_vision=[translation_error_vision,translation_vision-trans_actual];
                translation_error_NDI=[translation_error_NDI,translation_NDI-trans_actual];
    
    
    
                % disp("Translation Vision: ");
                % disp(translation_vision);
                % disp("Translation NDI: ");
                % disp(translation_NDI);
                Translation_Counter=Translation_Counter+1;

            end
            
        
    
        else %Rotation error

            %Amount of translation from vision
            rotation_vision=rad2deg(dist(quaternion(pose_vision(4:7)),quaternion(base_pose_vision(4:7))));

            %Amount of translation from NDI
            rotation_NDI=rad2deg(dist(quaternion(pose_NDI(4:7)),quaternion(base_pose_NDI(4:7))));

            % disp("Rotation Vision: ");
            % disp(rotation_vision);
            % disp("Rotation NDI: ");
            % disp(rotation_NDI);
    
    
        end

        %Update the base measurement for next loop around, we are always
        %comparing to last motion
        base_pose_NDI=pose_NDI;
        base_pose_vision=pose_vision;

    end

end



%% Plotting Translation errors

%Post process translation errors
translation_error_vision=abs(translation_error_vision)';

translation_error_NDI=abs(translation_error_NDI)';
const_NDI=mean(translation_error_NDI);
translation_error_NDI=abs(translation_error_NDI-const_NDI);

figure(1);
box_data=[translation_error_vision,translation_error_NDI];

%Grouping variable for x-axis
g1={'Vision Algorithm','NDI Tracker'};
boxplot(box_data,g1,'BoxStyle','filled','MedianStyle','target');
ylabel('Translation Error (mm)','FontWeight','bold','FontSize',12);
title('Comparison of Vision-Based and NDI Translation Tracking','FontSize',14,'FontWeight','bold')





>>>>>>> cb2dc77b51fe54e55eb17648b0794c2400e6f547
