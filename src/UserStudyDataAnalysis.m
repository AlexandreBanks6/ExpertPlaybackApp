%{
Description: Analyze data for the dV-FlexAR user study

%}

clear
clc
close all

%% Read in data
StudyRoot="E:/ExpertPlayback_Data_Fall2024/RawData/";
StudyOrderFile="E:/ExpertPlayback_Data_Fall2024/StudyOrder.xlsx";
study_order=readtable(StudyOrderFile);
study_order=table2cell(study_order);

%% One Handed Task First
onehanded_num_collisions=[];
onehanded_collision_duration=[];
onehanded_task_duration=[];
onehanded_clutch_presses=[];

AR_onehanded_num_collisions=[];
AR_onehanded_collision_duration=[];
AR_onehanded_task_duration=[];
AR_onehanded_clutch_presses=[];

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
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/PC1/Data_PC1_',num2str(k),'.csv');
                        data=xlsread(file_name);
                        num_collisions=[num_collisions,data(end,107)];
                        collision_duration=[collision_duration,data(end,108)];
                        task_duration=[task_duration,data(end,105)];
                        clutch_presses=[clutch_presses,sum(data(11:end,109))];
                    end
                    AR_onehanded_clutch_presses=[AR_onehanded_clutch_presses,mean(clutch_presses)];
                    AR_onehanded_num_collisions=[AR_onehanded_num_collisions,mean(num_collisions)];
                    AR_onehanded_collision_duration=[AR_onehanded_collision_duration,mean(collision_duration)];
                    AR_onehanded_task_duration=[AR_onehanded_task_duration,mean(task_duration)];

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
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/PC1/Data_PC1_',num2str(k),'.csv');
                        data=xlsread(file_name);
                        num_collisions=[num_collisions,data(end,107)];
                        collision_duration=[collision_duration,data(end,108)];
                        task_duration=[task_duration,data(end,105)];
                        clutch_presses=[clutch_presses,sum(data(11:end,109))];
                    end

                    onehanded_clutch_presses=[onehanded_clutch_presses,mean(clutch_presses)];
                    onehanded_num_collisions=[onehanded_num_collisions,mean(num_collisions)];
                    onehanded_collision_duration=[onehanded_collision_duration,mean(collision_duration)];
                    onehanded_task_duration=[onehanded_task_duration,mean(task_duration)];
                end
    
    
            end
    
        end
    end

end


%% Running Statistics

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

%******Calculating Descriptive Stats********

%AR
stats_AR_onehanded_num_collisions_avg=mean(AR_onehanded_num_collisions);
stats_AR_onehanded_num_collisions_std=std(AR_onehanded_num_collisions);

stats_AR_onehanded_collision_duration_avg=mean(AR_onehanded_collision_duration);
stats_AR_onehanded_collision_duration_std=std(AR_onehanded_collision_duration);

stats_AR_onehanded_task_duration_avg=mean(AR_onehanded_task_duration);
stats_AR_onehanded_task_duration_std=std(AR_onehanded_task_duration);

stats_AR_onehanded_collision_percentage_avg=mean(AR_onehanded_collision_percentage);
stats_AR_onehanded_collision_percentage_std=std(AR_onehanded_collision_percentage);

stats_AR_onehanded_collision_zscore_avg=mean(AR_onehanded_zscore);
stats_AR_onehanded_collision_zscore_std=std(AR_onehanded_zscore);

%no AR
stats_onehanded_num_collisions_avg=mean(onehanded_num_collisions);
stats_onehanded_num_collisions_std=std(onehanded_num_collisions);

stats_onehanded_collision_duration_avg=mean(onehanded_collision_duration);
stats_onehanded_collision_duration_std=std(onehanded_collision_duration);

stats_onehanded_task_duration_avg=mean(onehanded_task_duration);
stats_onehanded_task_duration_std=std(onehanded_task_duration);

stats_onehanded_collision_percentage_avg=mean(onehanded_collision_percentage);
stats_onehanded_collision_percentage_std=std(onehanded_collision_percentage);

stats_onehanded_collision_zscore_avg=mean(onehanded_zscore);
stats_onehanded_collision_zscore_std=std(onehanded_zscore);


%******************Running Tests***************
[is_normal,is_varequal,num_collisions_pval]=runComparison(AR_onehanded_num_collisions,onehanded_num_collisions,'Num Collisions');
[is_normal,is_varequal,collision_duration_pval]=runComparison(AR_onehanded_collision_duration,onehanded_collision_duration,'Collision Duration');
[is_normal,is_varequal,task_duration_pval]=runComparison(AR_onehanded_task_duration,onehanded_task_duration,'Task Duration');
[is_normal,is_varequal,collision_percentage_pval]=runComparison(AR_onehanded_collision_percentage,onehanded_collision_percentage,'Collision Percentage');
disp(['Mean p-val: ',num2str(mean([num_collisions_pval,collision_duration_pval,task_duration_pval,collision_percentage_pval]))]);
%% Displaying Results


%Grouping Variable
group=[repmat({'No AR'},length(onehanded_num_collisions),1);...
    repmat({'AR'},length(AR_onehanded_num_collisions),1)];

%Number of collisions
figure;
h=boxplot([onehanded_num_collisions';AR_onehanded_num_collisions'],group,'MedianStyle','line','BoxStyle','outline');
title('Cumulative Number of Collisions','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Collision #','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

%Collision Duration
figure;
h=boxplot([onehanded_collision_duration';AR_onehanded_collision_duration'],group,'MedianStyle','line','BoxStyle','outline');
title('Collision Duration','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Time (s)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

%Task Duration
figure;
h=boxplot([onehanded_task_duration';AR_onehanded_task_duration'],group,'MedianStyle','line','BoxStyle','outline');
title('Task Duration','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Time (s)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

%Number of Clutch Presses
figure;
h=boxplot([onehanded_clutch_presses';AR_onehanded_clutch_presses'],group,'MedianStyle','line','BoxStyle','outline');
title('Clutch Presses','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Clutch Press #','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

%Collision Duration as % of Task Duration
figure;
h=boxplot([onehanded_collision_percentage';AR_onehanded_collision_percentage'],group,'MedianStyle','line','BoxStyle','outline');
title('Collision Duration % of Total Duration','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Percentage (%)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

%Combined z-score
figure;
h=boxplot([onehanded_zscore';AR_onehanded_zscore'],group,'MedianStyle','line','BoxStyle','outline');
title('Combined Score','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('z-score ((score-mean)/std)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);


%% Analize as Funcion of Participant
%Convert list of participant designators to categorical
ARTrain_Partipants_Categorical=categorical(ARTrain_Participants);
NoAR_Participants_Categorical=categorical(NoAR_Participants);

%Plot # of collisions as function of participants
ymax=max([AR_onehanded_num_collisions,onehanded_num_collisions]);
figure;
plot(ARTrain_Partipants_Categorical,AR_onehanded_num_collisions,'ob',LineWidth=2);
title('AR # Of Collisions')
ylim([0,ymax]);

figure;
plot(NoAR_Participants_Categorical,onehanded_num_collisions,'ob',LineWidth=2);
title('No AR # Of Collisions')
ylim([0,ymax]);

%Plot Collision duration as a function of participants
ymax=max([AR_onehanded_collision_duration,onehanded_collision_duration]);
figure;
plot(ARTrain_Partipants_Categorical,AR_onehanded_collision_duration,'ob',LineWidth=2);
title('AR Collision Duration')
ylim([0,ymax]);

figure;
plot(NoAR_Participants_Categorical,onehanded_collision_duration,'ob',LineWidth=2);
title('No AR Collision Duration')
ylim([0,ymax]);

%Plot Task duration as a function of participants
ymax=max([AR_onehanded_task_duration,onehanded_task_duration]);
figure;
plot(ARTrain_Partipants_Categorical,AR_onehanded_task_duration,'ob',LineWidth=2);
title('AR Task Duration')
ylim([0,ymax]);

figure;
plot(NoAR_Participants_Categorical,onehanded_task_duration,'ob',LineWidth=2);
title('No AR Task Duration')
ylim([0,ymax]);



%% Function Definitions



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
            disp([data_type,': indepdent t-test P-val: ',num2str(p_val)]);
            
        else %Data do not have equal variances
            is_varequal=0;
            [h,p_val]=ttest2(AR_data,NoAR_data,'Vartype','unequal');
            disp([data_type,': Welch test P-val: ',num2str(p_val)]);
        end
    
    else %At least one dataset is not normally distributed
        is_normal=0;
        is_varequal=0;
        p_val=ranksum(AR_data,NoAR_data);
        disp([data_type,': Mann-Whitney U test P-val: ',num2str(p_val)]);
    end


end