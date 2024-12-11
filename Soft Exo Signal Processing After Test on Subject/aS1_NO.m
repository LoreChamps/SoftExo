close all;
clear;
clc;

% -------------------- MANUAL VALUES --------------------  %
subject = 'S1';

% ---------- MVC ---------- %
end_abduction_index_MVC = 3000; % index when the test goes abd to flex
EMG_abd_threshold = 250;
EMG_flex_threshold = 250;
% ---------- NO ---------- %
end_abduction_index_NO = 6150; % index when the test goes abd to flex
end_rom_abduction_NO = 3060;
end_rom_flexion_NO = 9100;
freq_fraction = 0.02;
start_idx_clean_NO = [];
end_idx_clean_NO = [];
threshold_elevation_NO = 10;
threshold_angle_tasks = 100;
samples4task = 1500;
imu_constant_rom = 1.4;
imu_constant_task = 1.3;
show_plot = 1;
rgb_NO = [0.9607843137254902 0.9607843137254902 0.9607843137254902];


%% ---------------------------- MVC: Import data from text file ---------------------------- %%
filename = [subject '_MVC'];

% path given the filename
path = ['C:\Users\loren\Desktop\TESTS\ICRA\' subject '\' filename '.csv'];

% ---------- Set up the Import Options and import the data ---------- %
opts = delimitedTextImportOptions("NumVariables", 15);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["EMG_flex_filt", "EMG_flex_raw", "EMG_abd_filt", "EMG_abd_raw", "EMG_baseline", "yaw1", "roll1", "pitch1", "angSpeed1", "linAcc1", "yaw2", "roll2", "pitch2", "angSpeed2", "linAcc2", "arm_calibrated", "flex_calibrated", "torso", "elevation", "flexion", "pressure1", "pref", "perr", "status_pump1", "status_pump2", "status_valve1", "status_valve2", "current_cycle", "needed_cycle", "micro_old", "micro_new", "macro_old", "macro_new", "control_mode", "time", "user_input", "kilos"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(path, opts);

% ---------- Clear temporary variables ---------- %
clear opts

% ---------- Data analisys ---------- %
EMG_abd_MVC = data.EMG_abd_filt(1:end_abduction_index_MVC);
EMG_flex_MVC = data.EMG_flex_filt(end_abduction_index_MVC:end);

% -------------------- Maximum value -------------------- %

% ----- abduction ----- %
contr = 1;
inside_contraction = 0;

idx_start_abd = zeros(1,3);
idx_end_abd = zeros(1,3);
max_mvc_abd = zeros(1,3);
idx_max_mvc_abd = zeros(1,3);

for ii = 1:length(EMG_abd_MVC)
    if contr <= 3
        if EMG_abd_MVC(ii) > EMG_abd_threshold && inside_contraction == 0
            idx_start_abd(contr) = ii;
            inside_contraction = 1;
        elseif EMG_abd_MVC(ii) < EMG_abd_threshold && inside_contraction == 1
            idx_end_abd(contr) = ii;
            inside_contraction = 0;
        end
        
        if idx_start_abd(contr)~=0 && idx_end_abd(contr)~=0
            [max_mvc_abd(contr), idx_max_mvc_abd(contr)] = max(EMG_abd_MVC(idx_start_abd(contr):idx_end_abd(contr)));
            contr = contr + 1;
        end
    end
end

idx_max_mvc_abd = idx_max_mvc_abd + idx_start_abd;
MVC_abd = mean(max_mvc_abd);

% ----- flexion ----- %
contr = 1;
inside_contraction = 0;

idx_start_flex = zeros(1,3);
idx_end_flex = zeros(1,3);
max_mvc_flex = zeros(1,3);
idx_max_mvc_flex = zeros(1,3);

for ii = 1:length(EMG_flex_MVC)
    if contr <= 3
        if EMG_flex_MVC(ii) > EMG_flex_threshold && inside_contraction == 0
            idx_start_flex(contr) = ii;
            inside_contraction = 1;
        elseif EMG_flex_MVC(ii) < EMG_flex_threshold && inside_contraction == 1
            idx_end_flex(contr) = ii;
            inside_contraction = 0;  
        end
    
        if idx_start_flex(contr)~=0 && idx_end_flex(contr)~=0
            [max_mvc_flex(contr), idx_max_mvc_flex(contr)] = max(EMG_flex_MVC(idx_start_flex(contr):idx_end_flex(contr)));
            contr = contr + 1;
        end
    end
end

idx_max_mvc_flex = idx_max_mvc_flex + idx_start_flex;
MVC_flex = mean(max_mvc_flex);


%% ---------------------------- NO: Import data from text file ---------------------------- %%

filename = [subject '_NO'];

% path given the filename
path = ['C:\Users\loren\Desktop\TESTS\ICRA\' subject '\' filename '.csv'];

% ---------- Set up the Import Options and import the data ---------- %
opts = delimitedTextImportOptions("NumVariables", 15);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["EMG_flex_filt", "EMG_flex_raw", "EMG_abd_filt", "EMG_abd_raw", "EMG_baseline", "yaw1", "roll1", "pitch1", "angSpeed1", "linAcc1", "yaw2", "roll2", "pitch2", "angSpeed2", "linAcc2", "arm_calibrated", "flex_calibrated", "torso", "elevation", "flexion", "pressure1", "pref", "perr", "status_pump1", "status_pump2", "status_valve1", "status_valve2", "current_cycle", "needed_cycle", "micro_old", "micro_new", "macro_old", "macro_new", "control_mode", "time", "user_input", "kilos"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(path, opts);

% ---------- Clear temporary variables ---------- %
clear opts

% ---------- Data analisys ---------- %
[b_l,a_l] = butter(3,freq_fraction,'low');
elev_filt = filtfilt(b_l,a_l,data.elevation);

if show_plot == 0
    if end_abduction_index_NO == 1
        title_elevation = 'Elevation (NO Robot)';
        title_EMG_abd = 'EMG abduction (NO Robot)';
        title_EMG_flex = 'EMG flexion (NO Robot)';
    
        figure(201)
        tiledlayout(3,1)
        ax1 = nexttile;
        plot(elev_filt,LineWidth=1)
        grid on;
        ylabel('Angle [deg]')
        ylim([0 200]);
        title(title_elevation,FontSize=14)
        ax2 = nexttile;
        plot(data.EMG_abd_raw,LineWidth=1)
        grid on;
        title(title_EMG_abd,FontSize=14)
        ax3 = nexttile;
        plot(data.EMG_flex_raw,LineWidth=1)
        grid on;
        title(title_EMG_flex,FontSize=14)
        linkaxes([ax1, ax2, ax3],'x');
    else
        title_elevation = 'ABDUCTION: Elevation (NO Robot)';
        title_EMG_abd = 'ABDUCTION: EMG (NO Robot)';
        
        figure(201)
        tiledlayout(2,1)
        ax1 = nexttile;
        plot(elev_filt(1:end_abduction_index_NO),LineWidth=1)
        grid on;
        ylabel('Angle [deg]')
        ylim([0 200]);
        title(title_elevation,FontSize=14)
        ax2 = nexttile;
        plot(data.EMG_abd_raw(1:end_abduction_index_NO),LineWidth=1)
        grid on;
        title(title_EMG_abd,FontSize=14)
        linkaxes([ax1, ax2],'x');
    
        title_elevation = 'FLEXION: Elevation (NO Robot)';
        title_EMG_flex = 'FLEXION: EMG (NO Robot)';
        
        figure(202)
        tiledlayout(2,1)
        ax1 = nexttile;
        plot(elev_filt(end_abduction_index_NO:end),LineWidth=1)
        grid on;
        ylabel('Angle [deg]')
        ylim([0 200]);
        title(title_elevation,FontSize=14)
        ax2 = nexttile;
        plot(data.EMG_flex_raw(end_abduction_index_NO:end),LineWidth=1)
        grid on;
        title(title_EMG_flex,FontSize=14)
        linkaxes([ax1, ax2],'x');
    end
end

% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ---------------------------------------- ABDUCTION ---------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %

% path given the filename
path = ['C:\Users\loren\Desktop\TESTS\ICRA\' subject '\' filename '.csv'];

% ---------- Set up the Import Options and import the data ---------- %
opts = delimitedTextImportOptions("NumVariables", 15);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["EMG_flex_filt", "EMG_flex_raw", "EMG_abd_filt", "EMG_abd_raw", "EMG_baseline", "yaw1", "roll1", "pitch1", "angSpeed1", "linAcc1", "yaw2", "roll2", "pitch2", "angSpeed2", "linAcc2", "arm_calibrated", "flex_calibrated", "torso", "elevation", "flexion", "pressure1", "pref", "perr", "status_pump1", "status_pump2", "status_valve1", "status_valve2", "current_cycle", "needed_cycle", "micro_old", "micro_new", "macro_old", "macro_new", "control_mode", "time", "user_input", "kilos"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(path, opts);

% ---------- Clear temporary variables ---------- %
clear opts

% ---------- Data analisys ---------- %
[b_l,a_l] = butter(3,freq_fraction,'low');
elev_filt = filtfilt(b_l,a_l,data.elevation);
% EMG Pre-processing & normalization
data.EMG_abd_filt = data.EMG_abd_filt / MVC_abd * 100;
data.EMG_flex_filt = data.EMG_flex_filt / MVC_flex * 100;

% Data manual correction
for ii = 1:length(start_idx_clean_NO)
    idx_clean_start = start_idx_clean_NO(ii);
    if ii == length(start_idx_clean_NO) && length(start_idx_clean_NO) ~= length(end_idx_clean_OFF)
        elev_filt(idx_clean_start:end) = 0;
    else
        idx_clean_end = end_idx_clean_OFF(ii);
        elev_filt(idx_clean_start:idx_clean_end) = 0;
    end
end

% selecting data for ABDUCTION
if end_abduction_index_NO ~= 0
    data{end_abduction_index_NO:end,:} = 0;
    elev_filt(end_abduction_index_NO:end) = 0;
end

% separation of tasks and resampling
tasks_idx = find(elev_filt>=threshold_elevation_NO);
new_motion_idx = [0; find(diff(tasks_idx)~=1); length(tasks_idx)-1]; % starting index to be added manually
new_motion_idx = new_motion_idx + 1;

n_tasks = length(new_motion_idx)-1;

% re-sized vectors
idx4task = cell(1,n_tasks); % array indices for each task
emg_resamp = cell(1,n_tasks);
elev_resamp = cell(1,n_tasks);
kilos4task = zeros(1,n_tasks); % array to match n° task with kilos lifted

for ii = 1:n_tasks
    tmp_task_idx = tasks_idx(new_motion_idx(ii)):tasks_idx(new_motion_idx(ii+1)-1);
    idx4task{ii} = tmp_task_idx;
    % kilos lifted in ii-th task
    mid_idx = round(median(tmp_task_idx));
    if elev_filt(mid_idx) > threshold_angle_tasks
        kilos4task(ii) = 1;
    else
        kilos4task(ii) = 0;
    end
    % re-sizing of vectors
    dt = length(tmp_task_idx)/(samples4task+1);
    tmp_new_idx = tmp_task_idx(1):dt:tmp_task_idx(end);
    if kilos4task(ii) == 1
        elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_rom;
    else
        elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_task;
    end
    emg_resamp{ii} = interp1(tmp_task_idx,data.EMG_abd_filt(idx4task{ii}),tmp_new_idx);
end

% adding extra elements at the end if the length of each task's vector is not the same
elev_resamp_matrix = zeros(n_tasks,samples4task);
emg_resamp_matrix = zeros(n_tasks,samples4task);
for ii = 1:n_tasks
    while length(emg_resamp{ii})~=samples4task
        emg_resamp{ii} = [emg_resamp{ii} emg_resamp{ii}(end)];
    end
    emg_resamp_matrix(ii,:) = emg_resamp{ii};
    while length(elev_resamp{ii})~=samples4task
        elev_resamp{ii} = [elev_resamp{ii} elev_resamp{ii}(end)];
    end
    elev_resamp_matrix(ii,:) = elev_resamp{ii};
end

% Plots
xvect = (1:samples4task)/samples4task*100;

if show_plot == 1
    for ii = 1:n_tasks
        if kilos4task(ii) == 1
            figure(203);
            sgtitle('ABDUCTION: Range of motion ROM')
        else
            figure(204);
            sgtitle('ABDUCTION: Range of motion ROM (90deg)')
        end

        % ELEVATION ANGLE SUBPLOTS %
        subplot(2,1,1)
        subplot_title = 'ABDUCTION: Elevation angle - NO ROBOT';
        if kilos4task(ii) == 1
            ylim([0 200]);
        else
            ylim([0 110]);
        end
        plot(xvect, elev_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)]);
        grid on;
        hold on;
        title(subplot_title);
        xlabel('Task percentage of completion')
        ylabel('Angle [deg]')
        xtickformat('percentage')
        legend;
        
        % EMG SUBPLOTS %
        subplot(2,1,2)
        subplot_title = 'ABDUCTION: EMG filtered - NO ROBOT';
        plot(xvect, emg_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
        grid on;
        hold on;
        title(subplot_title);
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
    end
end

for kilo = unique(kilos4task)
    % elevation angle average and std dev.
    current_kilos_idx = find(kilos4task==kilo);
    elev_avg = mean(elev_resamp_matrix(current_kilos_idx,:));
    elev_std = std(elev_resamp_matrix(current_kilos_idx,:));
    elev_up_lim = elev_avg + elev_std;
    elev_down_lim = elev_avg - elev_std;

    % EMG average and std dev.
    EMG_avg = mean(emg_resamp_matrix(current_kilos_idx,:));
    EMG_std = std(emg_resamp_matrix(current_kilos_idx,:));
    EMG_up_lim = EMG_avg + EMG_std;
    EMG_down_lim = EMG_avg - EMG_std;

    if show_plot == 1
        % update figures
        if kilo == 0
            figure(6);
            sgtitle('ABDUCTION: Tasks completed lifting 0 kg')
        else
            figure(7);
            sgtitle('ABDUCTION: Range of motion ROM')
        end
        x2 = [xvect, fliplr(xvect)];
        linename = 'avg NO';
        areaname = 'std NO';
        areacolor = rgb_NO;
        subplot(2,1,1)
        plot(xvect, elev_avg, 'Color', rgb_NO, LineWidth=2, DisplayName=linename);
        grid on
        hold on
        inBetween = [elev_down_lim, fliplr(elev_up_lim)];
        fill(x2, inBetween, areacolor, 'FaceAlpha', 0.3, 'DisplayName', areaname);
        title('ABDUCTION: ELEVATION - mean & std dev.')
        xlabel('Task percentage of completion')
        ylabel('Angle [deg]')
        if kilo == 1
            ylim([0 200]);
        else
            ylim([0 110]);
        end
        xtickformat('percentage')
        legend;
        subplot(2,1,2)
        plot(xvect, EMG_avg, 'Color', rgb_NO, LineWidth=2, DisplayName=linename);
        grid on
        hold on
        inBetween = [EMG_down_lim, fliplr(EMG_up_lim)];
        fill(x2, inBetween, areacolor, 'FaceAlpha', 0.3, 'DisplayName', areaname);
        title('ABDUCTION: EMG - mean & std dev.')
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
    end

    % saving relevant data
    tasks_idx_abduction_NO = tasks_idx;
    new_motion_idx_abduction_NO = new_motion_idx;
    n_tasks_abduction_NO = n_tasks;
    idx4task_abduction_NO = idx4task;
    emg_resamp_abduction_NO = emg_resamp;
    elev_resamp_abduction_NO = elev_resamp;
    kilos4task_abduction_NO = kilos4task;
end


% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ---------------------------------------- FLEXION ---------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %

% path given the filename
path = ['C:\Users\loren\Desktop\TESTS\ICRA\' subject '\' filename '.csv'];

% ---------- Set up the Import Options and import the data ---------- %
opts = delimitedTextImportOptions("NumVariables", 15);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["EMG_flex_filt", "EMG_flex_raw", "EMG_abd_filt", "EMG_abd_raw", "EMG_baseline", "yaw1", "roll1", "pitch1", "angSpeed1", "linAcc1", "yaw2", "roll2", "pitch2", "angSpeed2", "linAcc2", "arm_calibrated", "flex_calibrated", "torso", "elevation", "flexion", "pressure1", "pref", "perr", "status_pump1", "status_pump2", "status_valve1", "status_valve2", "current_cycle", "needed_cycle", "micro_old", "micro_new", "macro_old", "macro_new", "control_mode", "time", "user_input", "kilos"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(path, opts);

% ---------- Clear temporary variables ---------- %
clear opts

% ---------- Data analisys ---------- %
[b_l,a_l] = butter(3,freq_fraction,'low');
elev_filt = filtfilt(b_l,a_l,data.elevation);
% EMG Pre-processing & normalization
data.EMG_abd_filt = data.EMG_abd_filt / MVC_abd * 100;
data.EMG_flex_filt = data.EMG_flex_filt / MVC_flex * 100;

% Data manual correction
for ii = 1:length(start_idx_clean_NO)
    idx_clean_start = start_idx_clean_NO(ii);
    if ii == length(start_idx_clean_NO) && length(start_idx_clean_NO) ~= length(end_idx_clean_OFF)
        elev_filt(idx_clean_start:end) = 0;
    else
        idx_clean_end = end_idx_clean_OFF(ii);
        elev_filt(idx_clean_start:idx_clean_end) = 0;
    end
end

% selecting data for FLEXION
if end_abduction_index_NO ~= 0
    data{1:end_abduction_index_NO,:} = 0;
    elev_filt(1:end_abduction_index_NO) = 0;
end

% separation of tasks and resampling
tasks_idx = find(elev_filt>=threshold_elevation_NO);
new_motion_idx = [0; find(diff(tasks_idx)~=1); length(tasks_idx)-1]; % starting index to be added manually
new_motion_idx = new_motion_idx + 1;

n_tasks = length(new_motion_idx)-1;

% re-sized vectors
idx4task = cell(1,n_tasks); % array indices for each task
emg_resamp = cell(1,n_tasks);
elev_resamp = cell(1,n_tasks);
kilos4task = zeros(1,n_tasks); % array to match n° task with kilos lifted

for ii = 1:n_tasks
    tmp_task_idx = tasks_idx(new_motion_idx(ii)):tasks_idx(new_motion_idx(ii+1)-1);
    idx4task{ii} = tmp_task_idx;
    % kilos lifted in ii-th task
    mid_idx = round(median(tmp_task_idx));
    if elev_filt(mid_idx) > threshold_angle_tasks
        kilos4task(ii) = 1;
    else
        kilos4task(ii) = 0;
    end
    % re-sizing of vectors
    dt = length(tmp_task_idx)/(samples4task+1);
    tmp_new_idx = tmp_task_idx(1):dt:tmp_task_idx(end);
    if kilos4task(ii) == 1
        elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_rom;
    else
        elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_task;
    end
    emg_resamp{ii} = interp1(tmp_task_idx,data.EMG_flex_filt(idx4task{ii}),tmp_new_idx);
end

% adding extra elements at the end if the length of each task's vector is not the same
elev_resamp_matrix = zeros(n_tasks,samples4task);
emg_resamp_matrix = zeros(n_tasks,samples4task);
for ii = 1:n_tasks
    while length(emg_resamp{ii})~=samples4task
        emg_resamp{ii} = [emg_resamp{ii} emg_resamp{ii}(end)];
    end
    emg_resamp_matrix(ii,:) = emg_resamp{ii};
    while length(elev_resamp{ii})~=samples4task
        elev_resamp{ii} = [elev_resamp{ii} elev_resamp{ii}(end)];
    end
    elev_resamp_matrix(ii,:) = elev_resamp{ii};
end

% Plots
xvect = (1:samples4task)/samples4task*100;

if show_plot == 1
    for ii = 1:n_tasks
        if kilos4task(ii) == 1
            figure(205);
            sgtitle('FLEXION: Range of motion ROM')
        else
            figure(206);
            sgtitle('FLEXION: Range of motion ROM (90deg)')
        end

        % ELEVATION ANGLE SUBPLOTS %
        subplot(2,1,1)
        subplot_title = 'FLEXION: Elevation angle - NO ROBOT';
        if kilos4task(ii) == 1
            ylim([0 200]);
        else
            ylim([0 110]);
        end
        plot(xvect, elev_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)]);
        grid on;
        hold on;
        title(subplot_title);
        xlabel('Task percentage of completion')
        ylabel('Angle [deg]')
        xtickformat('percentage')
        legend;
        
        % EMG SUBPLOTS %
        subplot(2,1,2)
        subplot_title = 'FLEXION: EMG filtered - NO ROBOT';
        plot(xvect, emg_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
        grid on;
        hold on;
        title(subplot_title);
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
    end
end

for kilo = unique(kilos4task)
    % elevation angle average and std dev.
    current_kilos_idx = find(kilos4task==kilo);
    elev_avg = mean(elev_resamp_matrix(current_kilos_idx,:));
    elev_std = std(elev_resamp_matrix(current_kilos_idx,:));
    elev_up_lim = elev_avg + elev_std;
    elev_down_lim = elev_avg - elev_std;

    % EMG average and std dev.
    EMG_avg = mean(emg_resamp_matrix(current_kilos_idx,:));
    EMG_std = std(emg_resamp_matrix(current_kilos_idx,:));
    EMG_up_lim = EMG_avg + EMG_std;
    EMG_down_lim = EMG_avg - EMG_std;
    
    if show_plot == 1
        % update figures
        if kilo == 0
            figure(14);
            sgtitle('FLEXION: Tasks completed lifting 0 kg')
        else
            figure(15);
            sgtitle('FLEXION: Range of motion ROM')
        end
        x2 = [xvect, fliplr(xvect)];
        linename = 'avg NO';
        areaname = 'std NO';
        areacolor = rgb_NO;
        subplot(2,1,1)
        plot(xvect, elev_avg, 'Color', rgb_NO, LineWidth=2, DisplayName=linename);
        grid on
        hold on
        inBetween = [elev_down_lim, fliplr(elev_up_lim)];
        fill(x2, inBetween, areacolor, 'FaceAlpha', 0.3, 'DisplayName', areaname);
        title('FLEXION: ELEVATION - mean & std dev.')
        xlabel('Task percentage of completion')
        ylabel('Angle [deg]')
        if kilo == 1
            ylim([0 200]);
        else
            ylim([0 110]);
        end
        xtickformat('percentage')
        legend;
        subplot(2,1,2)
        plot(xvect, EMG_avg, 'Color', rgb_NO, LineWidth=2, DisplayName=linename);
        grid on
        hold on
        inBetween = [EMG_down_lim, fliplr(EMG_up_lim)];
        fill(x2, inBetween, areacolor, 'FaceAlpha', 0.3, 'DisplayName', areaname);
        title('FLEXION: EMG - mean & std dev.')
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
    end

    % saving relevant data
    tasks_idx_flexion_NO = tasks_idx;
    new_motion_idx_flexion_NO = new_motion_idx;
    n_tasks_flexion_NO = n_tasks;
    idx4task_flexion_NO = idx4task;
    emg_resamp_flexion_NO = emg_resamp;
    elev_resamp_flexion_NO = elev_resamp;
    kilos4task_flexion_NO = kilos4task;
end