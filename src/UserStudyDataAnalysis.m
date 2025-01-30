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

%Look at one-handed task first
onehanded_num_collisions=[];
onehanded_collision_duration=[];
onehanded_task_duration=[];

AR_onehanded_num_collisions=[];
AR_onehanded_collision_duration=[];
AR_onehanded_task_duration=[];

[row,col]=size(study_order);

for i=[1:row]   %Loops for all participants
    participant_type=study_order{i,2};
    participant_name=study_order{i,1};
    if strcmp(participant_name,'NaN')
        continue;
    else
        disp(["Participant Name: ",participant_name]);
        if strcmp(participant_type,'ARTrain')   %Enters if AR Train
    
            for j=[1:2] %Loops for two tasks
                if strcmp(study_order{i,j+2},'1Handed')   %Enters if 1-handed
                    %Grabs all five trials and finds mean of each outcome
                    num_collisions=[];
                    collision_duration=[];
                    task_duration=[];
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/PC1/Data_PC1_',num2str(k),'.csv');
                        data=xlsread(file_name);
                        num_collisions=[num_collisions,data(end,107)];
                        collision_duration=[collision_duration,data(end,108)];
                        task_duration=[task_duration,data(end,105)];
                    end

                    AR_onehanded_num_collisions=[AR_onehanded_num_collisions,mean(num_collisions)];
                    AR_onehanded_collision_duration=[AR_onehanded_collision_duration,mean(collision_duration)];
                    AR_onehanded_task_duration=[AR_onehanded_task_duration,mean(task_duration)];
                end
    
    
            end
    
        else %Control group
            for j=[1:2] %Loops for two tasks
                if strcmp(study_order{i,j+2},'1Handed')   %Enters if 1-handed
                    %Grabs all five trials and finds mean of each outcome
                    num_collisions=[];
                    collision_duration=[];
                    task_duration=[];
                    for k=[(j-1)*5+1:j*5]
                        file_name=strcat(StudyRoot,participant_name,'/PC1/Data_PC1_',num2str(k),'.csv');
                        data=xlsread(file_name);
                        num_collisions=[num_collisions,data(end,107)];
                        collision_duration=[collision_duration,data(end,108)];
                        task_duration=[task_duration,data(end,105)];
                    end

                    onehanded_num_collisions=[onehanded_num_collisions,mean(num_collisions)];
                    onehanded_collision_duration=[onehanded_collision_duration,mean(collision_duration)];
                    onehanded_task_duration=[onehanded_task_duration,mean(task_duration)];
                end
    
    
            end
    
        end
    end

end




%% Displaying Results

%calculating Collision Duration/Task Duration Percentage
AR_onehanded_collision_percentage=AR_onehanded_collision_duration./AR_onehanded_task_duration;
onehanded_collision_percentage=onehanded_collision_duration./onehanded_task_duration;

%Calculating the z-score
AR_onehanded_zscore=(AR_onehanded_num_collisions-mean(AR_onehanded_num_collisions))./std(AR_onehanded_num_collisions)+...
    (AR_onehanded_task_duration-mean(AR_onehanded_task_duration))./std(AR_onehanded_task_duration)+...
    (AR_onehanded_collision_duration-mean(AR_onehanded_collision_duration))./std(AR_onehanded_collision_duration);

onehanded_zscore=(onehanded_num_collisions-mean(onehanded_num_collisions))./std(onehanded_num_collisions)+...
    (onehanded_task_duration-mean(onehanded_task_duration))./std(onehanded_task_duration)+...
    (onehanded_collision_duration-mean(onehanded_collision_duration))./std(onehanded_collision_duration);

%Grouping Variable
group=[repmat({'No AR'},length(onehanded_num_collisions),1);...
    repmat({'AR'},length(AR_onehanded_num_collisions),1)];

figure;
h=boxplot([onehanded_num_collisions';AR_onehanded_num_collisions'],group,'MedianStyle','line','BoxStyle','outline');
title('Cumulative Number of Collisions','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Collision #','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

figure;
h=boxplot([onehanded_collision_duration';AR_onehanded_collision_duration'],group,'MedianStyle','line','BoxStyle','outline');
title('Collision Duration','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Time (s)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);


figure;
h=boxplot([onehanded_task_duration';AR_onehanded_task_duration'],group,'MedianStyle','line','BoxStyle','outline');
title('Task Duration','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Time (s)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);

figure;
h=boxplot([onehanded_collision_percentage';AR_onehanded_collision_percentage'],group,'MedianStyle','line','BoxStyle','outline');
title('Collision Duration % of Total Duration','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('Percentage (%)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);


figure;
h=boxplot([onehanded_zscore';AR_onehanded_zscore'],group,'MedianStyle','line','BoxStyle','outline');
title('Combined Score','FontName', 'Arial', 'FontSize', 14,'FontWeight','bold')
ylabel('z-score ((score-mean)/std)','FontName', 'Arial', 'FontSize', 14);
set(h, 'LineWidth', 2);