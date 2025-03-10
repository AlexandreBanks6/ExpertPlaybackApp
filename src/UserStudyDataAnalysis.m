%{
Description: Analyze data for the dV-FlexAR user study

%}

clear
clc
close all

%% Read in data

StudyRoot="C:/Users/alexa/OneDrive/Documents/UBC_Thesis/Data/ExpertPlayback_Data_Fall2024/OnlyPC1Data/";
StudyOrderFile="C:/Users/alexa/OneDrive/Documents/UBC_Thesis/Data/ExpertPlayback_Data_Fall2024/StudyOrder_V2.xlsx";
study_order=readtable(StudyOrderFile);
study_order=table2cell(study_order);
success_data=readtable("C:/Users/alexa/OneDrive/Documents/UBC_Thesis/Data/ExpertPlayback_Data_Fall2024/PickAndPlaceSuccesses.xlsx");
success_data=table2cell(success_data);
user_response_data=readtable("C:/Users/alexa/OneDrive/Documents/UBC_Thesis/Data/ExpertPlayback_Data_Fall2024/UserResponses_data.xlsx");
user_response_data=table2cell(user_response_data);
%% Sorting Data

%One Handed No AR
onehanded_num_collisions=[];
onehanded_collision_duration=[];
onehanded_task_duration=[];
onehanded_clutch_presses=[];
onehanded_segment_collision_duration=[];
onehanded_jerk={};
onehanded_psm1_time=[];
onehanded_psm3_time=[];
onehanded_handbalance_perpart=[];

%One Handed AR
AR_onehanded_num_collisions=[];
AR_onehanded_collision_duration=[];
AR_onehanded_task_duration=[];
AR_onehanded_clutch_presses=[];
AR_onehanded_segment_collision_duration=[];
AR_onehanded_jerk={};
AR_onehanded_psm1_time=[];
AR_onehanded_psm3_time=[];
AR_onehanded_handbalance_perpart=[];

%Pick and Place No AR
pickandplace_task_duration=[];
pickandplace_clutch_presses=[];
pickandplace_backpost_successes=[];
pickandplace_frontpost_successes=[];
pickandplace_total_successes=[];
pickandplace_psm1_time=[];
pickandplace_psm3_time=[];
pickandplace_handbalance_perpart=[];

%Pick and Place AR
AR_pickandplace_task_duration=[];
AR_pickandplace_clutch_presses=[];
AR_pickandplace_backpost_successes=[];
AR_pickandplace_frontpost_successes=[];
AR_pickandplace_total_successes=[];
AR_pickandplace_psm1_time=[];
AR_pickandplace_psm3_time=[];
AR_pickandplace_handbalance_perpart=[];

[row,col]=size(study_order);

%Getting Participant Names of which ones use AR and not
ARTrain_Participants={};
NoAR_Participants={};

for i=[1:row]   %Loops for all participants
    participant_type=study_order{i,2};
    participant_name=study_order{i,1};
    if strcmp(participant_name,'NaN')
        continue;
    else
        disp(["Participant Name: ",participant_name]);
        if strcmp(participant_type,'ARTrain')   %Enters if AR Train (experimental group)
            ARTrain_Participants=[ARTrain_Participants,participant_name]; %Updates the list of AR participant names
            for j=[1:2] %Loops for two tasks
                if strcmp(study_order{i,j+2},'1Handed')   %Enters if 1-handed
                    %Grabs all five trials and finds mean of each outcome
                    num_collisions=[];
                    collision_duration=[];
                    task_duration=[];
                    clutch_presses=[];
                    segment_collision_durations=[];
                    psm1_times=[];
                    psm3_times=[];

                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/Data_PC1_',num2str(k),'.csv');
                        if isfile(file_name)
                            data=xlsread(file_name);
                            num_collisions=[num_collisions,data(end,107)];
                            collision_duration=[collision_duration,data(end,108)];
                            task_duration=[task_duration,data(end,105)];
                            clutch_presses=[clutch_presses,computeNumClutchPresses(data(11:end,109))];
                            segment_collision_durations=[segment_collision_durations,meanSegmentDuration(data(11:end,106),data(11:end,108))];
                            [psm1_time,psm3_time]=computeEachPSMTime(data(11:end,6:8),data(11:end,19:21),data(11:end,1));
                            psm1_times=[psm1_times,psm1_time];
                            psm3_times=[psm3_times,psm3_time];
                            
                            %jerk_mag_psm1=computeJerkMag(data(11:end,6:8),data(11:end,1));
                            %jerk_mag_psm3=computeJerkMag(data(11:end,19:21),data(11:end,1));
                        end
                    end
                    AR_onehanded_psm1_time=[AR_onehanded_psm1_time,psm1_times];
                    AR_onehanded_psm3_time=[AR_onehanded_psm3_time,psm3_times];
                    AR_onehanded_handbalance_perpart=[AR_onehanded_handbalance_perpart,mean(abs(psm1_times-psm3_times))];


                    AR_onehanded_clutch_presses=[AR_onehanded_clutch_presses,mean(clutch_presses)];
                    AR_onehanded_num_collisions=[AR_onehanded_num_collisions,mean(num_collisions)];
                    AR_onehanded_collision_duration=[AR_onehanded_collision_duration,mean(collision_duration)];
                    AR_onehanded_task_duration=[AR_onehanded_task_duration,mean(task_duration)];
                    AR_onehanded_segment_collision_duration=[AR_onehanded_segment_collision_duration,segment_collision_durations];
                    %AR_onehanded_jerk=[AR_onehanded_jerk,max(jerk_mag_psm1,jerk_mag_psm3)];
                    % figure;
                    % plot([1:length(jerk_mag_psm1)],jerk_mag_psm1,'-b',[1:length(jerk_mag_psm3)],jerk_mag_psm3,'-r');
                    % title('AR');
                else %pick-and-place task
                    task_duration=[];
                    clutch_presses=[];
                    psm1_times=[];
                    psm3_times=[];
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/Data_PC1_',num2str(k),'.csv');
                        
                        if isfile(file_name)
                            data=xlsread(file_name);
                            task_duration=[task_duration,data(end,105)];
                            clutch_presses=[clutch_presses,computeNumClutchPresses(data(11:end,109))];
              
                            [psm1_time,psm3_time]=computeEachPSMTime(data(11:end,6:8),data(11:end,19:21),data(11:end,1));
                            psm1_times=[psm1_times,psm1_time];
                            psm3_times=[psm3_times,psm3_time];
                        end                        
                    end
                    AR_pickandplace_psm1_time=[AR_pickandplace_psm1_time,psm1_times];
                    AR_pickandplace_psm3_time=[AR_pickandplace_psm3_time,psm3_times];
                    AR_pickandplace_handbalance_perpart=[AR_pickandplace_handbalance_perpart,mean(abs(psm1_times-psm3_times))];

                    AR_pickandplace_task_duration=[AR_pickandplace_task_duration,mean(task_duration)];
                    AR_pickandplace_clutch_presses=[AR_pickandplace_clutch_presses,mean(clutch_presses)];

                    %Computes # of successes
                    success_data_idx=find(strcmp(success_data(:,1),participant_name)==1);
                    success_data_row=cell2mat(success_data(success_data_idx,2:end));
                    AR_pickandplace_backpost_successes=[AR_pickandplace_backpost_successes,mean(success_data_row(1:2:end),'omitnan')];
                    AR_pickandplace_frontpost_successes=[AR_pickandplace_frontpost_successes,mean(success_data_row(2:2:end),'omitnan')];
                    AR_pickandplace_total_successes=[AR_pickandplace_total_successes,mean(success_data_row(1:2:end)+success_data_row(2:2:end),'omitnan')];
                
                end
    
    
            end
    
        else %Control group
            NoAR_Participants=[NoAR_Participants,participant_name]; %Updates list of non ar participant names
            for j=[1:2] %Loops for two tasks
                if strcmp(study_order{i,j+2},'1Handed')   %Enters if 1-handed
                    %Grabs all five trials and finds mean of each outcome
                    num_collisions=[];
                    collision_duration=[];
                    task_duration=[];
                    clutch_presses=[];
                    segment_collision_durations=[];
                    psm1_times=[];
                    psm3_times=[];
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/Data_PC1_',num2str(k),'.csv');
                        if isfile(file_name)
                            data=xlsread(file_name);
                            num_collisions=[num_collisions,data(end,107)];
                            collision_duration=[collision_duration,data(end,108)];
                            task_duration=[task_duration,data(end,105)];
                            clutch_presses=[clutch_presses,computeNumClutchPresses(data(11:end,109))];
                            segment_collision_durations=[segment_collision_durations,meanSegmentDuration(data(11:end,106),data(11:end,108))];
                            [psm1_time,psm3_time]=computeEachPSMTime(data(11:end,6:8),data(11:end,19:21),data(11:end,1));
                            
                            psm1_times=[psm1_times,psm1_time];
                            psm3_times=[psm3_times,psm3_time];
                            
                            %jerk_mag_psm1=computeJerkMag(data(11:end,6:8),data(11:end,1));
                            %jerk_mag_psm3=computeJerkMag(data(11:end,19:21),data(11:end,1));
                        end

                    end

                    onehanded_psm1_time=[onehanded_psm1_time,psm1_times];
                    onehanded_psm3_time=[onehanded_psm3_time,psm3_times];
                    onehanded_handbalance_perpart=[onehanded_handbalance_perpart,mean(abs(psm1_times-psm3_times))];

                    onehanded_clutch_presses=[onehanded_clutch_presses,mean(clutch_presses)];
                    onehanded_num_collisions=[onehanded_num_collisions,mean(num_collisions)];
                    onehanded_collision_duration=[onehanded_collision_duration,mean(collision_duration)];
                    onehanded_task_duration=[onehanded_task_duration,mean(task_duration)];
                    onehanded_segment_collision_duration=[onehanded_segment_collision_duration,segment_collision_durations];
                    %onehanded_jerk=[onehanded_jerk,max(jerk_mag_psm1,jerk_mag_psm3)];

                    % figure;
                    % plot([1:length(jerk_mag_psm1)],jerk_mag_psm1,'-b',[1:length(jerk_mag_psm3)],jerk_mag_psm3,'-r');
                    % title('No AR');
                else %pick-and-place task
                    task_duration=[];
                    clutch_presses=[];
                    psm1_times=[];
                    psm3_times=[];
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/Data_PC1_',num2str(k),'.csv');
                        
                        if isfile(file_name)
                            data=xlsread(file_name);
                            task_duration=[task_duration,data(end,105)];
                            clutch_presses=[clutch_presses,computeNumClutchPresses(data(11:end,109))];                           
                            [psm1_time,psm3_time]=computeEachPSMTime(data(11:end,6:8),data(11:end,19:21),data(11:end,1));
                            psm1_times=[psm1_times,psm1_time];
                            psm3_times=[psm3_times,psm3_time];
                            
                        end                        
                    end
                    pickandplace_psm1_time=[pickandplace_psm1_time,psm1_times];
                    pickandplace_psm3_time=[pickandplace_psm3_time,psm3_times];

                    pickandplace_handbalance_perpart=[pickandplace_handbalance_perpart,mean(abs(psm1_times-psm3_times))];

                    pickandplace_task_duration=[pickandplace_task_duration,mean(task_duration)];
                    pickandplace_clutch_presses=[pickandplace_clutch_presses,mean(clutch_presses)];

                    %Computes # of successes
                    success_data_idx=find(strcmp(success_data(:,1),participant_name)==1);
                    success_data_row=cell2mat(success_data(success_data_idx,2:end));
                    pickandplace_backpost_successes=[pickandplace_backpost_successes,mean(success_data_row(1:2:end),'omitnan')];
                    pickandplace_frontpost_successes=[pickandplace_frontpost_successes,mean(success_data_row(2:2:end),'omitnan')];
                    pickandplace_total_successes=[pickandplace_total_successes,mean(success_data_row(1:2:end)+success_data_row(2:2:end),'omitnan')];
                
                end
    
    
            end
    
        end
    end

end


%% One-Handed Statistics

%********Calculating Collision Duration/Task Duration Percentage********
AR_onehanded_collision_percentage=(AR_onehanded_collision_duration./AR_onehanded_task_duration).*100;
onehanded_collision_percentage=(onehanded_collision_duration./onehanded_task_duration).*100;

%********Calculating the z-score*********
n_AR=length(ARTrain_Participants);
combined_collisions=[AR_onehanded_num_collisions,onehanded_num_collisions];
Z_score_num_collisions=(combined_collisions-mean(combined_collisions))./std(combined_collisions);

combined_task_duration=[AR_onehanded_task_duration,onehanded_task_duration];
Z_score_num_task_duration=(combined_task_duration-mean(combined_task_duration))./std(combined_task_duration);

combined_collision_duration=[AR_onehanded_collision_duration,onehanded_collision_duration];
Z_score_collision_duration=(combined_collision_duration-mean(combined_collision_duration))./std(combined_collision_duration);

AR_onehanded_zscore=Z_score_num_collisions(1:n_AR)+Z_score_num_task_duration(1:n_AR)+Z_score_collision_duration(1:n_AR);

onehanded_zscore=Z_score_num_collisions(n_AR+1:end)+Z_score_num_task_duration(n_AR+1:end)+Z_score_collision_duration(n_AR+1:end);

%Calculating the time difference between psm1 and psm3 for one_handed AR
%and non-ar

%One handed task
AR_onehanded_handbalance=abs(AR_onehanded_psm1_time-AR_onehanded_psm3_time);
onehanded_handbalance=abs(onehanded_psm1_time-onehanded_psm3_time);


%******************Running Tests***************
[is_normal,is_varequal,num_collisions_pval]=runComparison(AR_onehanded_num_collisions,onehanded_num_collisions,'1-Handed Num Collisions');
[is_normal,is_varequal,collision_duration_pval]=runComparison(AR_onehanded_collision_duration,onehanded_collision_duration,'1-Handed Collision Duration');
[is_normal,is_varequal,task_duration_pval]=runComparison(AR_onehanded_task_duration,onehanded_task_duration,'1-Handed Task Duration');
[is_normal,is_varequal,clutch_presses_pval]=runComparison(AR_onehanded_clutch_presses,onehanded_clutch_presses,'1-Handed Clutch Presses');
[is_normal,is_varequal,collision_percentage_pval]=runComparison(AR_onehanded_collision_percentage,onehanded_collision_percentage,'1-Handed Collision Percentage');
[is_normal,is_varequal,segment_duration_pval]=runComparison(AR_onehanded_segment_collision_duration,onehanded_segment_collision_duration,'1-Handed Collision Segment Duration');
[is_normal,is_varequal,zscore_pval]=runComparison(AR_onehanded_zscore,onehanded_zscore,'1-Handed ZScore');

%Hand Balance
[is_normal,is_varequal,handbalance_pval]=runComparison(AR_onehanded_handbalance,onehanded_handbalance,'1-Handed Hand Balance');
[is_normal,is_varequal,handbalance_pval_perpart]=runComparison(AR_onehanded_handbalance_perpart,onehanded_handbalance_perpart,'1-Handed Hand Balance Per Part');

disp(['1-Handed Mean p-val: ',num2str(mean([num_collisions_pval,collision_duration_pval,task_duration_pval,collision_percentage_pval,clutch_presses_pval,segment_duration_pval,zscore_pval]))]);

%% Pick-and-Place Statistics

%********Calculating the z-score*********
n_AR=length(ARTrain_Participants);
combined_task_duration=[AR_pickandplace_task_duration,pickandplace_task_duration];
Z_score_pickandplace_task_duration=(combined_task_duration-mean(combined_task_duration,"omitnan"))./std(combined_task_duration,"omitnan");

combined_num_fails=[10-AR_pickandplace_total_successes,10-pickandplace_total_successes];
Z_score_num_fails=(combined_num_fails-mean(combined_num_fails,"omitnan"))./std(combined_num_fails,"omitnan");

AR_pickandplace_zscore=Z_score_pickandplace_task_duration(1:n_AR)+Z_score_num_fails(1:n_AR);

pickandplace_zscore=Z_score_pickandplace_task_duration(n_AR+1:end)+Z_score_num_fails(n_AR+1:end);

%Time using hands:
%pick and place task
AR_pickandplace_handbalance=abs(AR_pickandplace_psm1_time-AR_pickandplace_psm3_time);
pickandplace_handbalance=abs(pickandplace_psm1_time-pickandplace_psm3_time);


%Running Stats
[is_normal,is_varequal,task_duration_pval]=runComparison(AR_pickandplace_task_duration,pickandplace_task_duration,'Pick and Place Task Duration');
[is_normal,is_varequal,clutch_presses_pval]=runComparison(AR_pickandplace_clutch_presses,pickandplace_clutch_presses,'Pick and Place Clutch Presses');

[is_normal,is_varequal,task_duration_pval]=runComparison(AR_pickandplace_backpost_successes,pickandplace_backpost_successes,'Pick and Place Back Post Success');
[is_normal,is_varequal,clutch_presses_pval]=runComparison(AR_pickandplace_frontpost_successes,pickandplace_frontpost_successes,'Pick and Place Front Post Success');
[is_normal,is_varequal,task_duration_pval]=runComparison(AR_pickandplace_total_successes,pickandplace_total_successes,'Pick and Place Total Success');

[is_normal,is_varequal,handbalance_pval]=runComparison(AR_pickandplace_handbalance,pickandplace_handbalance,'Pick and Place Hand Balance');
[is_normal,is_varequal,handbalance_pval]=runComparison(AR_pickandplace_handbalance_perpart,pickandplace_handbalance_perpart,'Pick and Place Hand Balance Per Part');

[is_normal,is_varequal,task_duration_pval]=runComparison(AR_pickandplace_zscore,pickandplace_zscore,'Pick and Place Z-Score');

%% Nasa TLX and Usability Scale
AR_train_indices=find(strcmp(study_order(:,2),'ARTrain')==1);
no_ar_indices=find(~(strcmp(study_order(:,2),'ARTrain')==1));
%Gets TLX data for AR
AR_onehanded_NASATLX=cell2mat(user_response_data((AR_train_indices-1)*2+1,3:8));
AR_onehanded_NASATLX_ForCombined=[AR_onehanded_NASATLX(:,1:3),10-AR_onehanded_NASATLX(:,4),AR_onehanded_NASATLX(:,5:end)]; %Change performance to be "non-performance"
%AR_onehanded_NASATLX_ForCombined=[AR_onehanded_NASATLX(:,1:3),AR_onehanded_NASATLX(:,5:end)];
AR_onehanded_Combined_NASATLX=sum(AR_onehanded_NASATLX_ForCombined,2);

AR_pickandplace_NASATLX=cell2mat(user_response_data((AR_train_indices-1)*2+2,3:8));
AR_pickandplace_NASATLX_ForCombined=[AR_pickandplace_NASATLX(:,1:3),10-AR_pickandplace_NASATLX(:,4),AR_pickandplace_NASATLX(:,5:end)];
%AR_pickandplace_NASATLX_ForCombined=[AR_pickandplace_NASATLX(:,1:3),AR_pickandplace_NASATLX(:,5:end)];
AR_pickandplace_Combined_NASATLX=sum(AR_pickandplace_NASATLX_ForCombined,2);

%UES Data for AR
AR_onehanded_UES=cell2mat(user_response_data((AR_train_indices-1)*2+1,9:end));
AR_pickandplace_UES=cell2mat(user_response_data((AR_train_indices-1)*2+2,9:end));

%Gets TLX data for No AR
onehanded_NASATLX=cell2mat(user_response_data((no_ar_indices-1)*2+1,3:8));
onehanded_NASATLX_ForCombined=[onehanded_NASATLX(:,1:3),10-onehanded_NASATLX(:,4),onehanded_NASATLX(:,5:end)];
%onehanded_NASATLX_ForCombined=[onehanded_NASATLX(:,1:3),onehanded_NASATLX(:,5:end)];
onehanded_Combined_NASATLX=sum(onehanded_NASATLX_ForCombined,2);

pickandplace_NASATLX=cell2mat(user_response_data((no_ar_indices-1)*2+2,3:8));
pickandplace_NASATLX_ForCombined=[pickandplace_NASATLX(:,1:3),10-pickandplace_NASATLX(:,4),pickandplace_NASATLX(:,5:end)];
%pickandplace_NASATLX_ForCombined=[pickandplace_NASATLX(:,1:3),pickandplace_NASATLX(:,5:end)];
pickandplace_Combined_NASATLX=sum(pickandplace_NASATLX_ForCombined,2);

%UES Data for no AR
onehanded_UES=cell2mat(user_response_data((no_ar_indices-1)*2+1,9:end));
pickandplace_UES=cell2mat(user_response_data((no_ar_indices-1)*2+2,9:end));


%% Nasa TLX Stats

TLX_Categories={'Mental Demand','Physical Demand','Temporal Demand','Performance','Effort','Frustration'};

%Loops through each separate One Handed NASA-TLX and computes stats
for(i=[1:6])
    [is_normal,is_varequal,handbalance_pval]=runComparison(AR_onehanded_NASATLX(:,i),onehanded_NASATLX(:,i),['One-Handed NASA TLX - ',TLX_Categories{i}]);
end

%Combined TLX score
[is_normal,is_varequal,handbalance_pval]=runComparison(AR_onehanded_Combined_NASATLX,onehanded_Combined_NASATLX,'One-Handed Combined NASA TLX');

%Pick and Place
for(i=[1:6])
    [is_normal,is_varequal,handbalance_pval]=runComparison(AR_pickandplace_NASATLX(:,i),pickandplace_NASATLX(:,i),['Pick and Place NASA TLX - ',TLX_Categories{i}]);
end

[is_normal,is_varequal,handbalance_pval]=runComparison(AR_pickandplace_Combined_NASATLX,pickandplace_Combined_NASATLX,'Pick and Place Combined NASA TLX');


%% Plotting One-Handed Results
figuresaveroot='C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Code\dvFlexAR_PaperFigures\';

%z-score
singleComparisonBoxPlot(AR_onehanded_zscore,onehanded_zscore,[figuresaveroot,'userstudy_boxplot_onehanded_zscore.svg'],"Combined Z-Score",0);
%Task Duration
singleComparisonBoxPlot(AR_onehanded_task_duration,onehanded_task_duration,[figuresaveroot,'userstudy_boxplot_onehanded_taskduration.svg'],"Task Duration (s)",1);
%Num Collisions
singleComparisonBoxPlot(AR_onehanded_num_collisions,onehanded_num_collisions,[figuresaveroot,'userstudy_boxplot_onehanded_numcollisions.svg'],"# Collisions",0);
%Collision Duration
singleComparisonBoxPlot(AR_onehanded_collision_duration,onehanded_collision_duration,[figuresaveroot,'userstudy_boxplot_onehanded_collisionduration.svg'],"Collision Duration (s)",0);
%Collision Segment Duration
singleComparisonBoxPlot(AR_onehanded_segment_collision_duration,onehanded_segment_collision_duration,[figuresaveroot,'userstudy_boxplot_onehanded_collisionsegmentduration.svg'],"Collision Segment Duration (s)",1);
%Hand Balance
singleComparisonBoxPlot(AR_onehanded_handbalance_perpart,onehanded_handbalance_perpart,[figuresaveroot,'userstudy_boxplot_onehanded_handbalance.svg'],"|PSM1 Time - PSM3 Time| (s)",1);

%Plotting NASA TLX Results
plotNasaTLXResults(AR_onehanded_NASATLX,onehanded_NASATLX,AR_onehanded_Combined_NASATLX,onehanded_Combined_NASATLX,[figuresaveroot,'userstudy_boxplot_onehanded_NASATLX.svg'],[0,0,0,0,0,1,0],16);
%% Plotting Pick and Place Results
figuresaveroot='C:\Users\alexa\OneDrive\Documents\UBC_Thesis\Code\dvFlexAR_PaperFigures\';

%z-score
singleComparisonBoxPlot(AR_pickandplace_zscore,pickandplace_zscore,[figuresaveroot,'userstudy_boxplot_pickandplace_zscore.svg'],"Combined Z-Score",1);
%Task Duration
singleComparisonBoxPlot(AR_pickandplace_task_duration,pickandplace_task_duration,[figuresaveroot,'userstudy_boxplot_pickandplace_taskduration.svg'],"Task Duration (s)",0);
%Num Collisions
singleComparisonBoxPlot(AR_pickandplace_clutch_presses,pickandplace_clutch_presses,[figuresaveroot,'userstudy_boxplot_pickandplace_clutchpresses.svg'],"# Clutch Presses",0);
%Hand Balance
singleComparisonBoxPlot(AR_pickandplace_handbalance_perpart,pickandplace_handbalance_perpart,[figuresaveroot,'userstudy_boxplot_pickandplace_handbalance.svg'],"|PSM1 Time - PSM3 Time| (s)",1);

%Plotting NASA TLX Results
plotNasaTLXResults(AR_pickandplace_NASATLX,pickandplace_NASATLX,AR_pickandplace_Combined_NASATLX,pickandplace_Combined_NASATLX,[figuresaveroot,'userstudy_boxplot_pickandplace_NASATLX.svg'],[1,0,0,0,0,1,1],12);

%Plotting "Successes"
plotSuccessesPickAndPlace([AR_pickandplace_total_successes',AR_pickandplace_frontpost_successes',AR_pickandplace_backpost_successes'], ...
    [pickandplace_total_successes',pickandplace_frontpost_successes',pickandplace_backpost_successes'], ...
    [figuresaveroot,'userstudy_boxplot_pickandplace_successes.svg'],[1,1,1]);

%% Function Definitions

%Data processing function that finds mean duration of segments
function mean_segment_duration=meanSegmentDuration(collision_binary,collision_time_durations)

startIdxs=find(diff([0;collision_binary])==1); %Segment start
endIdxs=find(diff([collision_binary;0])==-1); %Segment end

durations=collision_time_durations(endIdxs)-collision_time_durations(startIdxs);
mean_segment_duration=mean(durations);
end

function jerk_mag=computeJerkMag(position_array,time_vector)
    valid_indices=~any(isnan(position_array), 2) & ~isnan(time_vector);
    position_array=position_array(valid_indices,:);
    time_vector=time_vector(valid_indices);
    velocity=diff(position_array,[],1)./diff(time_vector);
    acceleration=diff(velocity,[],1)./diff(time_vector(1:end-1));
    %jerk=diff(acceleration,[],1)./diff(time_vector(1:end-2));
    jerk_mag=vecnorm(acceleration,2,2);
end

function num_clutch_presses=computeNumClutchPresses(clutch_press_binary)
diff_arr=diff(clutch_press_binary);
num_clutch_presses=sum(diff_arr==1);

end

function [psm1_time,psm3_time]=computeEachPSMTime(position_array_psm1,position_array_psm3,time_vector)
   % valid_indices=~any(isnan(position_array_psm1), 2) & ~any(isnan(position_array_psm3), 2) & ~isnan(time_vector);
   % position_array_psm1=position_array_psm1(valid_indices,:);
   % position_array_psm3=position_array_psm3(valid_indices,:);
    %time_vector=time_vector(valid_indices);
    dt=diff(time_vector);
    velocity_psm1=diff(position_array_psm1,[],1)./dt;
    velocity_psm3=diff(position_array_psm3,[],1)./dt;

    speed_psm1=vecnorm(velocity_psm1,2,2);
    speed_psm3=vecnorm(velocity_psm3,2,2);

    psm_used=speed_psm1>speed_psm3; %Binary vector, 1 if psm1 is faster, else 0 if psm3 is faster

    psm1_time=sum(dt(psm_used));%,"omitnan");
    psm3_time=sum(dt(~psm_used));%,"omitnan");    
end

%Stats (Checks for normality, equal variances, and then runs the t-test)
function [is_normal,is_varequal,p_val]=runComparison(AR_data,NoAR_data,data_type)
    [h1, p1] = swtest(AR_data,0.05);
    [h2, p2] = swtest(NoAR_data,0.05);

    %[h1, p1] = kstest(AR_data);
    %[h2, p2] = kstest(NoAR_data);

    if((~h1)&&(~h2)) %Both datasets are normally distributed
        is_normal=1;
        h=vartest2(AR_data,NoAR_data); %Check if equal variance
        if(~h) %Data have equal variances
            is_varequal=1;
            %Run the indepdent t-test
            [h,p_val]=ttest2(AR_data,NoAR_data);
            disp([data_type,' AR=',num2str(mean(AR_data,"omitnan")),' STD=',num2str(std(AR_data,"omitnan")),' No AR=',num2str(mean(NoAR_data,"omitnan")),' STD=',num2str(std(NoAR_data,"omitnan")),' indepdent t-test P-val: ',num2str(p_val)]);
            
        else %Data do not have equal variances
            is_varequal=0;
            [h,p_val]=ttest2(AR_data,NoAR_data,'Vartype','unequal');
            disp([data_type,' AR=',num2str(mean(AR_data,"omitnan")),' STD=',num2str(std(AR_data,"omitnan")),' No AR=',num2str(mean(NoAR_data,"omitnan")),' STD=',num2str(std(NoAR_data,"omitnan")),' Welch test P-val: ',num2str(p_val)]);
        end
    
    else %At least one dataset is not normally distributed
        is_normal=0;
        is_varequal=0;
        p_val=ranksum(AR_data,NoAR_data);
        disp([data_type,' AR=',num2str(mean(AR_data,"omitnan")),' STD=',num2str(std(AR_data,"omitnan")),' No AR=',num2str(mean(NoAR_data,"omitnan")),' STD=',num2str(std(NoAR_data,"omitnan")),' Mann-Whitney U test P-val: ',num2str(p_val)]);
    end


end

function singleComparisonBoxPlot(AR_data,No_AR_data,savefile,yl,show_significance)
%AR_data_outlier_index=findIndicesOfExtremeOutliersForDisplay(AR_data);
%No_AR_data_outlier_index=findIndicesOfExtremeOutliersForDisplay(No_AR_data);
AR_data=AR_data';
No_AR_data=No_AR_data';
%Two boxplots with ar and no ar groups
positions = [0.25 0.75]; %Space

% Define the number of columns
numCols = size(AR_data, 2);

% Create grouping variables for boxplot
group = [];  % Stores group identifiers
%positions = [];  % Stores x positions of boxes
data = [];  % Stores all data points

% Assign positions manually (keeping spacing)
for i = 1:numCols
    % Append left group data
    data = [data; AR_data(:, i)];
    group = [group; repmat(i, size(AR_data, 1), 1)];
    %positions = [positions; 0.25 + (i-1)*0.25];  % Adjust spacing
    
    % Append right group data
    data = [data; No_AR_data(:, i)];
    group = [group; repmat(i+numCols, size(No_AR_data, 1), 1)];
    %positions = [positions; 1.5 + (i-1)*0.25];  % Adjust spacing
end

%Different shades of grey
dark_gray = [0.3, 0.3, 0.3];
light_gray = [0.6, 0.6, 0.6];
color=[light_gray;dark_gray];

% min_val=min([min(data)-0.5,0]);
% max_val=max(data)+0.5;
% n_ticks=


figure;
set(gcf, 'Color', 'w');
boxplot(data,group,'Positions',positions,'Colors',color,'BoxStyle','filled','MedianStyle','target','Symbol','r+');%'BoxStyle','filled','MedianStyle','target','Symbol','r+');

%Formatting
xticks(positions);
set(gca, 'FontSize', 8);
xticklabels({'AR','No AR'});
ax=gca;
ax.XAxis.FontSize=12;
ax.XAxis.FontName='Times';
ax.XAxis.FontWeight='bold';
ylabel(yl,'FontSize', 12, 'FontName', 'Times', 'FontWeight', 'bold');

set(gcf, 'Position', [100, 100, 150, 240]);  % (left, bottom, width, height in pixels)
% Determine Y-axis limits
upperWhisker_AR = computeUpperWhiskers(AR_data);
upperWhisker_No_AR = computeUpperWhiskers(No_AR_data);
ymax=max([upperWhisker_AR,upperWhisker_No_AR]);
yOffset = (ymax - min(data)) * 0.15;
yl=ylim;
ylim([yl(1),ymax + yOffset])

% Add significance marker if requested
if show_significance
    hold on;
    % Define the position for the significance line
    yLine = ymax + yOffset * 0.5;
    line([positions(1) positions(2)], [yLine yLine], 'Color', 'k', 'LineWidth', 1.5);
    text(mean(positions), yLine + yOffset * 0.15, '*', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    hold off;
end

saveas(gcf, savefile);

end

function outlier_indices=findIndicesOfExtremeOutliersForDisplay(dat)
sdv=std(dat);
avg=mean(dat);
outlier_indices=(dat>(avg+2*sdv))|(dat<(avg-2*sdv));
end

function upperWhisker = computeUpperWhiskers(data)
    Q1 = prctile(data, 25);  % First quartile
    Q3 = prctile(data, 75);  % Third quartile
    IQR = Q3 - Q1;              % Interquartile range
    valswithin_whiskers=data(data<=(Q3 + 1.5*IQR));
    upperWhisker = max(valswithin_whiskers);
end


function plotNasaTLXResults(AR_NASATLX,No_AR_NASATLX,AR_NASATLX_Total,No_AR_NASATLX_Total,savefile,sig_vec,fontsize)
positions=[0.25 0.5 1 1.25 1.75 2 2.5 2.75 3.25 3.5 4 4.25 4.75 5]; %Position of boxplots
AR_data=[AR_NASATLX./10,AR_NASATLX_Total./60];
No_AR_data=[No_AR_NASATLX./10,No_AR_NASATLX_Total./60];
grp_labs={'Mental Demand','Physical Demand','Temporal Demand','Performance','Effort','Frustration','Total'};

numCols = size(AR_data, 2);
% Create grouping variables for boxplot
group = [];  % Stores group identifiers
%positions = [];  % Stores x positions of boxes
data = [];  % Stores all data points

% Assign positions manually (keeping spacing)
for i = 1:numCols
    % Append AR group data
    data = [data; AR_data(:, i)];
    group = [group; repmat((i-1)*2, size(AR_data, 1), 1)];
    %positions = [positions; 0.25 + (i-1)*0.25];  % Adjust spacing
    
    % Append No AR group data
    data = [data; No_AR_data(:, i)];
    group = [group; repmat((i-1)*2+1, size(No_AR_data, 1), 1)];
    %positions = [positions; 1.5 + (i-1)*0.25];  % Adjust spacing
end
dark_gray = [0.3, 0.3, 0.3];
light_gray = [0.6, 0.6, 0.6];
color=[];
for i=[1:6]
    color=[color;light_gray;dark_gray];
end

figure;
set(gcf, 'Color', 'w');
boxplot(data,group,'Positions',positions,'Colors',color,'BoxStyle','filled','MedianStyle','target','Symbol','r+');%'BoxStyle','filled','MedianStyle','target','Symbol','r+');

%Formatting
xticks([mean(positions(1:2)),mean(positions(3:4)),mean(positions(5:6)),mean(positions(7:8)),mean(positions(9:10)),mean(positions(11:12)),mean(positions(13:14))]);
set(gca, 'FontSize', (3/4)*fontsize);
xticklabels(grp_labs);
ax=gca;
ax.XAxis.FontSize=fontsize;
ax.XAxis.FontName='Times';
ax.XAxis.FontWeight='bold';
ylabel('Normalized NASA TLX Score','FontSize', fontsize, 'FontName', 'Times', 'FontWeight', 'bold');

set(gcf, 'Position', [100, 100, 700, 560]);  % (left, bottom, width, height in pixels)

hold on;

yLine=1-0.027;
%Add significance annotations
for i=[1:length(sig_vec)]
    if(sig_vec(i))
    line([positions((i-1)*2+1),positions((i-1)*2+2)], [yLine yLine], 'Color', 'k', 'LineWidth', 1.5);
    text(mean([positions((i-1)*2+1),positions((i-1)*2+2)]), yLine + 0.005, '*', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    end

end

hLegend = gobjects(2,1); % Preallocate legend handles

legend_labels = {'AR','No AR'}; % Legend names

for j = 1:2 % Only 2 different colors, corresponding to each variable
    hLegend(j) = plot(nan, nan, 's', 'MarkerFaceColor', color(j,:), 'MarkerEdgeColor', 'k');
end

legend(hLegend, legend_labels, 'Location', 'southwest','FontSize', fontsize, 'FontName', 'Times');


hold off;
saveas(gcf, savefile);

end

function plotSuccessesPickAndPlace(AR_Successes,No_AR_Successes,savefile,sig_vec)
positions=[0.25 0.5 1 1.25 1.75 2]; %Position of boxplots
%Normalizes successes
AR_Successes(:,1)=AR_Successes(:,1)./10.*100;
AR_Successes(:,2)=AR_Successes(:,2)./6.*100;
AR_Successes(:,3)=AR_Successes(:,3)./4.*100;

No_AR_Successes(:,1)=No_AR_Successes(:,1)./10.*100;
No_AR_Successes(:,2)=No_AR_Successes(:,2)./6.*100;
No_AR_Successes(:,3)=No_AR_Successes(:,3)./4.*100;
grp_labs={'Total Success','Front Posts Success','Back Posts Success'};

numCols = size(AR_Successes, 2);
% Create grouping variables for boxplot
group = [];  % Stores group identifiers
%positions = [];  % Stores x positions of boxes
data = [];  % Stores all data points

% Assign positions manually (keeping spacing)
for i = 1:numCols
    % Append AR group data
    data = [data; AR_Successes(:, i)];
    group = [group; repmat((i-1)*2, size(AR_Successes, 1), 1)];
    %positions = [positions; 0.25 + (i-1)*0.25];  % Adjust spacing
    
    % Append No AR group data
    data = [data; No_AR_Successes(:, i)];
    group = [group; repmat((i-1)*2+1, size(No_AR_Successes, 1), 1)];
    %positions = [positions; 1.5 + (i-1)*0.25];  % Adjust spacing
end
dark_gray = [0.3, 0.3, 0.3];
light_gray = [0.6, 0.6, 0.6];
color=[];
for i=[1:3]
    color=[color;light_gray;dark_gray];
end

figure;
set(gcf, 'Color', 'w');
boxplot(data,group,'Positions',positions,'Colors',color,'BoxStyle','filled','MedianStyle','target','Symbol','r+');%'BoxStyle','filled','MedianStyle','target','Symbol','r+');

%Formatting
xticks([mean(positions(1:2)),mean(positions(3:4)),mean(positions(5:6))]);
set(gca, 'FontSize', 8);
xticklabels(grp_labs);
ax=gca;
ax.XAxis.FontSize=12;
ax.XAxis.FontName='Times';
ax.XAxis.FontWeight='bold';
ylabel('Success Rate (%)','FontSize', 12, 'FontName', 'Times', 'FontWeight', 'bold');

set(gcf, 'Position', [100, 100, 375, 560]);  % (left, bottom, width, height in pixels)

hold on;

yLine=100.5;
%Add significance annotations
for i=[1:length(sig_vec)]
    if(sig_vec(i))
    line([positions((i-1)*2+1),positions((i-1)*2+2)], [yLine yLine], 'Color', 'k', 'LineWidth', 1.5);
    text(mean([positions((i-1)*2+1),positions((i-1)*2+2)]), yLine + 0.5, '*', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    end

end

hLegend = gobjects(2,1); % Preallocate legend handles

legend_labels = {'AR','No AR'}; % Legend names

for j = 1:2 % Only 2 different colors, corresponding to each variable
    hLegend(j) = plot(nan, nan, 's', 'MarkerFaceColor', color(j,:), 'MarkerEdgeColor', 'k');
end

legend(hLegend, legend_labels, 'Location', 'southwest','FontSize', 12, 'FontName', 'Times');


hold off;
saveas(gcf, savefile);

end
