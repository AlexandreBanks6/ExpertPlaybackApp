ac_path='C:/Users/alexa/OneDrive/Documents/UBC_Thesis/Code/ExpertPlaybackApp/resources/Calib/Calib_Best/API_Error_Offset/p_lc_ac_list.npy';
rep_path='C:/Users/alexa/OneDrive/Documents/UBC_Thesis/Code/ExpertPlaybackApp/resources/Calib/Calib_Best/API_Error_Offset/p_lc_rep_list.npy';

ac_data=readNPY(ac_path);
ac_data=ac_data.*100;
rep_data=readNPY(rep_path);
rep_data=rep_data.*100;
X_axis=1:11;


%Plot results
figure;
subplot(3,2,1);
plot(X_axis,ac_data(:,1),'bo-',X_axis,rep_data(:,1),'go-');
ylabel('x (cm)');

mean_x_error=mean(abs(ac_data(:,1)-rep_data(:,1)));
mean_line=repmat(mean_x_error,11);
subplot(3,2,2);
plot(X_axis,abs(ac_data(:,1)-rep_data(:,1)),'ro-',X_axis,mean_line,'b--');
ylabel('x error (cm)');

subplot(3,2,3);
plot(X_axis,ac_data(:,2),'bo-',X_axis,rep_data(:,2),'go-');
ylabel('y (cm)');


mean_y_error=mean(abs(ac_data(:,2)-rep_data(:,2)));
mean_line=repmat(mean_y_error,11);
subplot(3,2,4);
plot(X_axis,abs(ac_data(:,2)-rep_data(:,2)),'ro-',X_axis,mean_line,'b--');
ylabel('y error (cm)');

subplot(3,2,5);
plot(X_axis,ac_data(:,3),'bo-',X_axis,rep_data(:,3),'go-');
ylabel('z (cm)');

mean_z_error=mean(abs(ac_data(:,3)-rep_data(:,3)));
mean_line=repmat(mean_z_error,11);
subplot(3,2,6);
plot(X_axis,abs(ac_data(:,3)-rep_data(:,3)),'ro-',X_axis,mean_line,'b--');
ylabel('z error (cm)');
