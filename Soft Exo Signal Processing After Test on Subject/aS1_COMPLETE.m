close all;
clear;

% -------------------- MANUAL VALUES --------------------  %
subject = 'S1';
save_data = 0; % if want to save data in .mat file
show_plot = 0; % if plot has to be showed - DEBUGGER
show_separation = 0; % if plot dynamic/static has to be showed - DEBUGGER

% ---------- MVC ---------- %
end_abduction_index_MVC = 3000; % index when the test goes abd to flex
EMG_abd_threshold = 250;
EMG_flex_threshold = 250;
check_plot = 0;

% ---------- NO ---------- %
end_abduction_index_NO = 6150; % index when the test goes abd to flex
end_rom_abduction_NO = 3060;
end_rom_flexion_NO = 9100;
start_idx_clean_NO = [];
end_idx_clean_NO = [];
threshold_elevation_NO = 10;
threshold_angle_tasks = 100;
rgb_NO = [0.9607843137254902 0.9607843137254902 0.9607843137254902];

% ---------- Data ---------- %
freq_fraction = 0.02; % filtering fraction frequency for elevation angle
samples4task = 1500; % number of desired samples for each task
threshold_elevation_OFF = 10; % trigger value to detect elevation motion
threshold_elevation_ON = 10; % trigger value to detect elevation motion
end_abduction_index_OFF = 26939; % index when the test goes abd to flex
end_abduction_index_ON = 24922; % index when the test goes abd to flex
imu_constant_rom = 1.4; % scaling factor for ROM exercise
imu_constant_task = 1.3; % scaling factor for task exercises
start_idx_clean_OFF = [5466, 10599, 13330, 22673, 33562, 40741, 50343];
end_idx_clean_OFF = [5618, 11596, 16238, 30502, 33946, 44259];
start_idx_clean_ON = [1, 9447, 11298, 21260, 33338, 49577];
end_idx_clean_ON = [4702, 9952, 12399, 26975, 43281];
rgb_OFF = [0.9725490196078431 0.807843137254902 0.8];
rgb_ON = [0.6901960784313725 0.8901960784313725 0.9019607843137255];




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

if show_plot == 0
    figure(101)
    tiledlayout(2,1)
    ax1 = nexttile;
    plot(EMG_abd_MVC,LineWidth=1)
    hold on;
    grid on;
    title('MVC abduction',FontSize=14)
    ax2 = nexttile;
    plot(EMG_flex_MVC,LineWidth=1)
    hold on;
    grid on;
    title('MVC flexion',FontSize=14)
    linkaxes([ax1, ax2],'x');
end

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

% check plot
if check_plot == 1
    figure(102)
    tiledlayout(2,1)
    ax1 = nexttile;
    plot(EMG_abd_MVC,LineWidth=1)
    hold on;
    grid on;
    stem(idx_max_mvc_abd,max_mvc_abd,LineWidth=1)
    title('MVC abduction',FontSize=14)
    ax2 = nexttile;
    plot(EMG_flex_MVC,LineWidth=1)
    hold on;
    grid on;
    stem(idx_max_mvc_flex,max_mvc_flex,LineWidth=1)
    title('MVC flexion',FontSize=14)
    linkaxes([ax1, ax2],'x');

    % pause to check the plot
    pause
end



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
        title_EMG_abd = 'ABDUCTION: MID EMG (NO Robot)';
        
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
        title_EMG_flex = 'FLEXION: ANT EMG (NO Robot)';
        
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
    if ii == length(start_idx_clean_NO) && length(start_idx_clean_NO) ~= length(end_idx_clean_NO)
        elev_filt(idx_clean_start:end) = 0;
    else
        idx_clean_end = end_idx_clean_NO(ii);
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
emgMID_resamp = cell(1,n_tasks);
emgANT_resamp = cell(1,n_tasks);
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
    emgMID_resamp{ii} = interp1(tmp_task_idx,data.EMG_abd_filt(idx4task{ii}),tmp_new_idx);
    emgANT_resamp{ii} = interp1(tmp_task_idx,data.EMG_flex_filt(idx4task{ii}),tmp_new_idx);
end

% adding extra elements at the end if the length of each task's vector is not the same
elev_resamp_matrix = zeros(n_tasks,samples4task);
emgMID_resamp_matrix = zeros(n_tasks,samples4task);
emgANT_resamp_matrix = zeros(n_tasks,samples4task);
for ii = 1:n_tasks
    while length(emgMID_resamp{ii})~=samples4task
        emgMID_resamp{ii} = [emgMID_resamp{ii} emgMID_resamp{ii}(end)];
    end
    emgMID_resamp_matrix(ii,:) = emgMID_resamp{ii};
    while length(emgANT_resamp{ii})~=samples4task
        emgANT_resamp{ii} = [emgANT_resamp{ii} emgANT_resamp{ii}(end)];
    end
    emgANT_resamp_matrix(ii,:) = emgANT_resamp{ii};
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
        subplot(3,1,1)
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
        subplot(3,1,2)
        subplot_title = 'ABDUCTION: MID EMG filtered - NO ROBOT';
        plot(xvect, emgMID_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
        grid on;
        hold on;
        title(subplot_title);
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
        subplot(3,1,3)
        subplot_title = 'ABDUCTION: ANT EMG filtered - NO ROBOT';
        plot(xvect, emgANT_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
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

    % EMG MID average and std dev.
    EMG_avg = mean(emgMID_resamp_matrix(current_kilos_idx,:));
    EMG_std = std(emgMID_resamp_matrix(current_kilos_idx,:));
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
        title('ABDUCTION: MID EMG - mean & std dev.')
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
    end
end

% saving relevant data
tasks_idx_abduction_NO = tasks_idx;
new_motion_idx_abduction_NO = new_motion_idx;
n_tasks_abduction_NO = n_tasks;
idx4task_abduction_NO = idx4task;
emgMID_resamp_abduction_NO = emgMID_resamp;
emgANT_resamp_abduction_NO = emgANT_resamp;
elev_resamp_abduction_NO = elev_resamp;
kilos4task_abduction_NO = kilos4task;

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
    if ii == length(start_idx_clean_NO) && length(start_idx_clean_NO) ~= length(end_idx_clean_NO)
        elev_filt(idx_clean_start:end) = 0;
    else
        idx_clean_end = end_idx_clean_NO(ii);
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
emgMID_resamp = cell(1,n_tasks);
emgANT_resamp = cell(1,n_tasks);
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
    emgMID_resamp{ii} = interp1(tmp_task_idx,data.EMG_abd_filt(idx4task{ii}),tmp_new_idx);
    emgANT_resamp{ii} = interp1(tmp_task_idx,data.EMG_flex_filt(idx4task{ii}),tmp_new_idx);
end

% adding extra elements at the end if the length of each task's vector is not the same
elev_resamp_matrix = zeros(n_tasks,samples4task);
emgMID_resamp_matrix = zeros(n_tasks,samples4task);
emgANT_resamp_matrix = zeros(n_tasks,samples4task);
for ii = 1:n_tasks
    while length(emgMID_resamp{ii})~=samples4task
        emgMID_resamp{ii} = [emgMID_resamp{ii} emgMID_resamp{ii}(end)];
    end
    emgMID_resamp_matrix(ii,:) = emgMID_resamp{ii};
    while length(emgANT_resamp{ii})~=samples4task
        emgANT_resamp{ii} = [emgANT_resamp{ii} emgANT_resamp{ii}(end)];
    end
    emgANT_resamp_matrix(ii,:) = emgANT_resamp{ii};
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
        subplot(3,1,1)
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
        subplot(3,1,2)
        subplot_title = 'FLEXION: MID EMG filtered - NO ROBOT';
        plot(xvect, emgMID_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
        grid on;
        hold on;
        title(subplot_title);
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
        subplot(3,1,3)
        subplot_title = 'FLEXION: ANT EMG filtered - NO ROBOT';
        plot(xvect, emgANT_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
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
    EMG_avg = mean(emgANT_resamp_matrix(current_kilos_idx,:));
    EMG_std = std(emgANT_resamp_matrix(current_kilos_idx,:));
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
        title('FLEXION: ANT EMG - mean & std dev.')
        xlabel('Task percentage of completion')
        ylabel('EMG level [% MVC]')
        xtickformat('percentage')
        ytickformat('percentage')
        legend;
    end
end

% saving relevant data
tasks_idx_flexion_NO = tasks_idx;
new_motion_idx_flexion_NO = new_motion_idx;
n_tasks_flexion_NO = n_tasks;
idx4task_flexion_NO = idx4task;
emgMID_resamp_flexion_NO = emgMID_resamp;
emgANT_resamp_flexion_NO = emgANT_resamp;
elev_resamp_flexion_NO = elev_resamp;
kilos4task_flexion_NO = kilos4task;


%% ---------------------------- DATA: Import data from text file --------------------------- %%

% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ---------------------------------------- ABDUCTION ---------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
% ------------------------------------------------------------------------------------------- %
for robot_ON = [0,1]
    if robot_ON == 1
        filename = [subject '_ON'];
    else
        filename = [subject '_OFF'];
    end
    
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
    t_vect = (data.time - data.time(1))* 1e-3;
    
    % IMU Pre-processing & normalization
    [b_l,a_l] = butter(3,freq_fraction,'low');
    elev_filt = filtfilt(b_l,a_l,data.elevation);
    kilos_cat = categorical(data.kilos, [1 0 2], {'ROM', '0 Kg', '2 Kg'});

    % EMG Pre-processing & normalization
    data.EMG_abd_filt = data.EMG_abd_filt / MVC_abd * 100;
    data.EMG_flex_filt = data.EMG_flex_filt / MVC_flex * 100;

    if show_plot == 0
        if robot_ON == 0
            figure(1009)
            title_elevation = 'Elevation (Robot OFF)';
            title_EMG_abd = 'ABDUCTION: MID EMG (Robot OFF)';
            title_EMG_flex = 'FLEXION: ANT EMG (Robot OFF)';
        else
            figure(5013)
            sgtitle('ROBOT ON')
            title_elevation = 'Elevation (Robot ON)';
            title_EMG_abd = 'ABDUCTION: MID EMG (Robot ON)';
            title_EMG_flex = 'FLEXION: ANT EMG (Robot ON)';
        end
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
    end
    
    % Data manual correction
    if robot_ON == 0
        for ii = 1:length(start_idx_clean_OFF)
            idx_clean_start = start_idx_clean_OFF(ii);
            if ii == length(start_idx_clean_OFF) && length(start_idx_clean_OFF) ~= length(end_idx_clean_OFF)
                elev_filt(idx_clean_start:end) = 0;
            else
                idx_clean_end = end_idx_clean_OFF(ii);
                elev_filt(idx_clean_start:idx_clean_end) = 0;
            end
        end
        threshold_elevation = threshold_elevation_OFF;
        end_abduction_index = end_abduction_index_OFF;
    else
        for ii = 1:length(start_idx_clean_ON)
            idx_clean_start = start_idx_clean_ON(ii);
            if ii == length(start_idx_clean_ON) && length(start_idx_clean_ON) ~= length(end_idx_clean_ON)
                elev_filt(idx_clean_start:end) = 0;
            else
                idx_clean_end = end_idx_clean_ON(ii);
                elev_filt(idx_clean_start:idx_clean_end) = 0;
            end
        end
        threshold_elevation = threshold_elevation_ON;
        end_abduction_index = end_abduction_index_ON;
    end

    % selecting data for ABDUCTION
    if end_abduction_index ~= 0
        data{end_abduction_index:end,:} = 0;
        elev_filt(end_abduction_index:end) = 0;
    end
    
    if show_plot == 0
        if robot_ON == 0
            figure(1)
            title_elevation = 'ABDUCTION: Arm Elevation - filtered (Robot OFF)';
            title_EMG = 'ABDUCTION: MID EMG level (Robot OFF)';
            title_weight = 'ABDUCTION: Weigth (Robot OFF)';
        else
            figure(5)
            title_elevation = 'ABDUCTION: Arm Elevation - filtered (Robot ON)';
            title_EMG = 'ABDUCTION: MID EMG level (Robot ON)';
            title_weight = 'ABDUCTION: Weigth (Robot ON)';
        end

        tiledlayout(3,1) % change according to number of plots to show
        ax1 = nexttile;
        plot(elev_filt,LineWidth=1)
        grid on;
        ylabel('Angle [deg]')
        ylim([0 200]);
        title(title_elevation,FontSize=14)
        ax2 = nexttile;
        plot(data.EMG_abd_raw,LineWidth=1)
        grid on;
        title(title_EMG,FontSize=14)
        ax3 = nexttile;
        plot(kilos_cat,LineWidth=1)
        grid on;
        title(title_weight,FontSize=14)
        linkaxes([ax1, ax2, ax3],'x');
    end
    
    % separation of tasks and resampling
    tasks_idx = find(elev_filt>=threshold_elevation);
    new_motion_idx = [0; find(diff(tasks_idx)~=1); length(tasks_idx)-1]; % starting index to be added manually
    new_motion_idx = new_motion_idx + 1;
    
    n_tasks = length(new_motion_idx)-1;
    
    % re-sized vectors
    idx4task = cell(1,n_tasks); % array indices for each task
    emgMID_resamp = cell(1,n_tasks);
    emgANT_resamp = cell(1,n_tasks);
    elev_resamp = cell(1,n_tasks);
    kilos4task = zeros(1,n_tasks); % array to match n° task with kilos lifted
    
    for ii = 1:n_tasks
        tmp_task_idx = tasks_idx(new_motion_idx(ii)):tasks_idx(new_motion_idx(ii+1)-1);
        idx4task{ii} = tmp_task_idx;
        % kilos lifted in ii-th task
        mid_idx = round(median(tmp_task_idx));
        kilos4task(ii) = data.kilos(mid_idx);
        % re-sizing of vectors
        dt = length(tmp_task_idx)/(samples4task+1);
        tmp_new_idx = tmp_task_idx(1):dt:tmp_task_idx(end);
        if kilos4task(ii) == 1
            elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_rom;
        else
            elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_task;
        end
        emgMID_resamp{ii} = interp1(tmp_task_idx,data.EMG_abd_filt(idx4task{ii}),tmp_new_idx);
        emgANT_resamp{ii} = interp1(tmp_task_idx,data.EMG_flex_filt(idx4task{ii}),tmp_new_idx);
    end
    
    % adding extra elements at the end if the length of each task's vector is not the same
    elev_resamp_matrix = zeros(n_tasks,samples4task);
    emgMID_resamp_matrix = zeros(n_tasks,samples4task);
    emgANT_resamp_matrix = zeros(n_tasks,samples4task);
    for ii = 1:n_tasks
        while length(emgMID_resamp{ii})~=samples4task
            emgMID_resamp{ii} = [emgMID_resamp{ii} emgMID_resamp{ii}(end)];
        end
        emgMID_resamp_matrix(ii,:) = emgMID_resamp{ii};
        while length(emgANT_resamp{ii})~=samples4task
            emgANT_resamp{ii} = [emgANT_resamp{ii} emgANT_resamp{ii}(end)];
        end
        emgANT_resamp_matrix(ii,:) = emgANT_resamp{ii};
        while length(elev_resamp{ii})~=samples4task
            elev_resamp{ii} = [elev_resamp{ii} elev_resamp{ii}(end)];
        end
        elev_resamp_matrix(ii,:) = elev_resamp{ii};
    end
    
    % Plots
    xvect = (1:samples4task)/samples4task*100;
    
    if show_plot == 1
        for ii = 1:n_tasks
            if kilos4task(ii) == 0
                figure(2);
                sgtitle('ABDUCTION: Tasks completed lifting 0 kg')
            elseif kilos4task(ii) == 1
                figure(3);
                sgtitle('ABDUCTION: Range of motion ROM')
            else
                figure(4);
                sgtitle('ABDUCTION: Tasks completed lifting 2 kg')
            end
            
            % ELEVATION ANGLE SUBPLOTS %
            if robot_ON == 0
                subplot_title = 'ABDUCTION: Elevation filtered - ROBOT OFF';
                if kilos4task(ii) == 1 % ROM has no case robot ON
                    subplot(2,1,1)
                    ylim([0 200]);
                else
                    subplot(2,2,1)
                    ylim([0 110]);
                end
            else
                subplot(2,2,2)
                ylim([0 110]);
                subplot_title = 'ABDUCTION: Elevation angle - ROBOT ON';
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
            if robot_ON == 0
                subplot_title = 'ABDUCTION: MID EMG filtered - ROBOT OFF';
                if kilos4task(ii) == 1 % ROM has no case robot ON
                    subplot(2,1,2)
                else
                    subplot(2,2,3)
                end
                
            else
                subplot(2,2,4)
                subplot_title = 'ABDUCTION: MID EMG filtered - ROBOT ON';
            end
            plot(xvect, emgMID_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
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
        EMG_avg = mean(emgMID_resamp_matrix(current_kilos_idx,:));
        EMG_std = std(emgMID_resamp_matrix(current_kilos_idx,:));
        EMG_up_lim = EMG_avg + EMG_std;
        EMG_down_lim = EMG_avg - EMG_std;
        
        if show_plot == 1
            % update figures
            if kilo == 0
                figure(6);
                sgtitle('ABDUCTION: Tasks completed lifting 0 kg')
            elseif kilo == 1
                figure(7);
                sgtitle('ABDUCTION: Range of motion ROM')
            else
                figure(8);
                sgtitle('ABDUCTION: Tasks completed lifting 2 kg')
            end
            x2 = [xvect, fliplr(xvect)];
            if robot_ON == 0
                rgb = rgb_OFF;
                linename = 'avg OFF';
                areaname = 'std OFF';
            else
                rgb = rgb_ON;
                linename = 'avg ON';
                areaname = 'std ON';
            end
            subplot(2,1,1)
            plot(xvect, elev_avg, 'Color', rgb, LineWidth=2, DisplayName=linename);
            grid on
            hold on
            inBetween = [elev_down_lim, fliplr(elev_up_lim)];
            fill(x2, inBetween, rgb, 'FaceAlpha', 0.3, 'DisplayName', areaname);
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
            plot(xvect, EMG_avg, 'Color', rgb, LineWidth=2, DisplayName=linename);
            grid on
            hold on
            inBetween = [EMG_down_lim, fliplr(EMG_up_lim)];
            fill(x2, inBetween, rgb, 'FaceAlpha', 0.3, 'DisplayName', areaname);
            title('ABDUCTION: MID EMG - mean & std dev.')
            xlabel('Task percentage of completion')
            ylabel('EMG level [% MVC]')
            xtickformat('percentage')
            ytickformat('percentage')
            legend;
        end
    end

    % saving relevant data
    if robot_ON == 0
        tasks_idx_abduction_OFF = tasks_idx;
        new_motion_idx_abduction_OFF = new_motion_idx;
        n_tasks_abduction_OFF = n_tasks;
        idx4task_abduction_OFF = idx4task;
        emgMID_resamp_abduction_OFF = emgMID_resamp;
        emgANT_resamp_abduction_OFF = emgANT_resamp;
        elev_resamp_abduction_OFF = elev_resamp;
        kilos4task_abduction_OFF = kilos4task;
    else
        tasks_idx_abduction_ON = tasks_idx;
        new_motion_idx_abduction_ON = new_motion_idx;
        n_tasks_abduction_ON = n_tasks;
        idx4task_abduction_ON = idx4task;
        emgMID_resamp_abduction_ON = emgMID_resamp;
        emgANT_resamp_abduction_ON = emgANT_resamp;
        elev_resamp_abduction_ON = elev_resamp;
        kilos4task_abduction_ON = kilos4task;
    end
end

% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ---------------------------------------- FLEXION ---------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %
% ----------------------------------------------------------------------------------------- %

for robot_ON = [0,1]
    if robot_ON == 1
        filename = [subject '_ON'];
    else
        filename = [subject '_OFF'];
    end
    
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
    t_vect = (data.time - data.time(1))* 1e-3;
    
    % IMU Pre-processing & normalization
    [b_l,a_l] = butter(3,freq_fraction,'low');
    elev_filt = filtfilt(b_l,a_l,data.elevation);
    kilos_cat = categorical(data.kilos, [1 0 2], {'ROM', '0 Kg', '2 Kg'});

    % EMG Pre-processing & normalization
    data.EMG_abd_filt = data.EMG_abd_filt / MVC_abd * 100;
    data.EMG_flex_filt = data.EMG_flex_filt / MVC_flex * 100;
    
    % Data manual correction
    if robot_ON == 0
        for ii = 1:length(start_idx_clean_OFF)
            idx_clean_start = start_idx_clean_OFF(ii);
            if ii == length(start_idx_clean_OFF) && length(start_idx_clean_OFF) ~= length(end_idx_clean_OFF)
                elev_filt(idx_clean_start:end) = 0;
            else
                idx_clean_end = end_idx_clean_OFF(ii);
                elev_filt(idx_clean_start:idx_clean_end) = 0;
            end
        end
        threshold_elevation = threshold_elevation_OFF;
        end_abduction_index = end_abduction_index_OFF;
    else
        for ii = 1:length(start_idx_clean_ON)
            idx_clean_start = start_idx_clean_ON(ii);
            if ii == length(start_idx_clean_ON) && length(start_idx_clean_ON) ~= length(end_idx_clean_ON)
                elev_filt(idx_clean_start:end) = 0;
            else
                idx_clean_end = end_idx_clean_ON(ii);
                elev_filt(idx_clean_start:idx_clean_end) = 0;
            end
        end
        threshold_elevation = threshold_elevation_ON;
        end_abduction_index = end_abduction_index_ON;
    end

    % selecting data for FLEXION
    if end_abduction_index ~= 0
        data{1:end_abduction_index,:} = 0;
        elev_filt(1:end_abduction_index) = 0;
    end
    
    if show_plot == 0
        if robot_ON == 0
            figure(9)
            title_elevation = 'FLEXION: Arm Elevation - filtered (Robot OFF)';
            title_EMG = 'FLEXION: ANT EMG level (Robot OFF)';
            title_weight = 'FLEXION: Weigth (Robot OFF)';
        else
            figure(13)
            title_elevation = 'FLEXION: Arm Elevation - filtered (Robot ON)';
            title_EMG = 'FLEXION: ANT EMG level (Robot ON)';
            title_weight = 'FLEXION: Weigth (Robot ON)';
        end
        
        tiledlayout(3,1)
        ax1 = nexttile;
        plot(elev_filt,LineWidth=1)
        grid on;
        ylabel('Angle [deg]')
        ylim([0 200]);
        title(title_elevation,FontSize=14)
        ax2 = nexttile;
        plot(data.EMG_flex_raw,LineWidth=1)
        grid on;
        title(title_EMG,FontSize=14)
        ax3 = nexttile;
        plot(kilos_cat,LineWidth=1)
        grid on;
        title(title_weight,FontSize=14)
        linkaxes([ax1, ax2, ax3],'x');
    end
    
    % separation of tasks and resampling
    tasks_idx = find(elev_filt>=threshold_elevation);
    new_motion_idx = [0; find(diff(tasks_idx)~=1); length(tasks_idx)-1]; % starting index to be added manually
    new_motion_idx = new_motion_idx + 1;
    
    n_tasks = length(new_motion_idx)-1;
    
    % re-sized vectors
    idx4task = cell(1,n_tasks); % array indices for each task
    emgMID_resamp = cell(1,n_tasks);
    emgANT_resamp = cell(1,n_tasks);
    elev_resamp = cell(1,n_tasks);
    kilos4task = zeros(1,n_tasks); % array to match n° task with kilos lifted
    
    for ii = 1:n_tasks
        tmp_task_idx = tasks_idx(new_motion_idx(ii)):tasks_idx(new_motion_idx(ii+1)-1);
        idx4task{ii} = tmp_task_idx;
        % kilos lifted in ii-th task
        mid_idx = round(median(tmp_task_idx));
        kilos4task(ii) = data.kilos(mid_idx);
        % re-sizing of vectors
        dt = length(tmp_task_idx)/(samples4task+1);
        tmp_new_idx = tmp_task_idx(1):dt:tmp_task_idx(end);
        if kilos4task(ii) == 1
            elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_rom;
        else
            elev_resamp{ii} = interp1(tmp_task_idx,elev_filt(idx4task{ii}),tmp_new_idx) * imu_constant_task;
        end
        emgMID_resamp{ii} = interp1(tmp_task_idx,data.EMG_abd_filt(idx4task{ii}),tmp_new_idx);
        emgANT_resamp{ii} = interp1(tmp_task_idx,data.EMG_flex_filt(idx4task{ii}),tmp_new_idx);
    end
    
    % adding extra elements at the end if the length of each task's vector is not the same
    elev_resamp_matrix = zeros(n_tasks,samples4task);
    emgMID_resamp_matrix = zeros(n_tasks,samples4task);
    emgANT_resamp_matrix = zeros(n_tasks,samples4task);
    for ii = 1:n_tasks
        while length(emgMID_resamp{ii})~=samples4task
            emgMID_resamp{ii} = [emgMID_resamp{ii} emgMID_resamp{ii}(end)];
        end
        emgMID_resamp_matrix(ii,:) = emgMID_resamp{ii};
        while length(emgANT_resamp{ii})~=samples4task
            emgANT_resamp{ii} = [emgANT_resamp{ii} emgANT_resamp{ii}(end)];
        end
        emgANT_resamp_matrix(ii,:) = emgANT_resamp{ii};
        while length(elev_resamp{ii})~=samples4task
            elev_resamp{ii} = [elev_resamp{ii} elev_resamp{ii}(end)];
        end
        elev_resamp_matrix(ii,:) = elev_resamp{ii};
    end
    
    % Plots
    xvect = (1:samples4task)/samples4task*100;
    
    if show_plot == 1
        for ii = 1:n_tasks
            if kilos4task(ii) == 0
                figure(10);
                sgtitle('FLEXION: Tasks completed lifting 0 kg')
            elseif kilos4task(ii) == 1
                figure(11);
                sgtitle('FLEXION: Range of motion ROM')
            else
                figure(12);
                sgtitle('FLEXION: Tasks completed lifting 2 kg')
            end
            
            % ELEVATION ANGLE SUBPLOTS %
            if robot_ON == 0
                subplot_title = 'FLEXION: Elevation angle - ROBOT OFF';
                if kilos4task(ii) == 1 % ROM has no case robot ON
                    subplot(2,1,1)
                    ylim([0 200]);
                else
                    subplot(2,2,1)
                    ylim([0 110]);
                end
            else
                subplot(2,2,2)
                ylim([0 110]);
                subplot_title = 'FLEXION: Elevation angle - ROBOT ON';
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
            if robot_ON == 0
                subplot_title = 'FLEXION: ANT EMG filtered - ROBOT OFF';
                if kilos4task(ii) == 1 % ROM has no case robot ON
                    subplot(2,1,2)
                else
                    subplot(2,2,3)
                end
            else
                subplot(2,2,4)
                subplot_title = 'FLEXION: ANT EMG filtered - ROBOT ON';
            end
            plot(xvect, emgANT_resamp{ii}, 'DisplayName', ['elev: ' num2str(ii)])
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
        EMG_avg = mean(emgANT_resamp_matrix(current_kilos_idx,:));
        EMG_std = std(emgANT_resamp_matrix(current_kilos_idx,:));
        EMG_up_lim = EMG_avg + EMG_std;
        EMG_down_lim = EMG_avg - EMG_std;
        
        if show_plot == 1
            % update figures
            if kilo == 0
                figure(14);
                sgtitle('FLEXION: Tasks completed lifting 0 kg')
            elseif kilo == 1
                figure(15);
                sgtitle('FLEXION: Range of motion ROM')
            else
                figure(16);
                sgtitle('FLEXION: Tasks completed lifting 2 kg')
            end
            x2 = [xvect, fliplr(xvect)];
            if robot_ON == 0
                rgb = rgb_OFF;
                linename = 'avg OFF';
                areaname = 'std OFF';
            else
                rgb = rgb_ON;
                linename = 'avg ON';
                areaname = 'std ON';
            end
            subplot(2,1,1)
            plot(xvect, elev_avg, 'Color', rgb, LineWidth=2, DisplayName=linename);
            grid on
            hold on
            inBetween = [elev_down_lim, fliplr(elev_up_lim)];
            fill(x2, inBetween, rgb, 'FaceAlpha', 0.3, 'DisplayName', areaname);
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
            plot(xvect, EMG_avg, 'Color', rgb, LineWidth=2, DisplayName=linename);
            grid on
            hold on
            inBetween = [EMG_down_lim, fliplr(EMG_up_lim)];
            fill(x2, inBetween, rgb, 'FaceAlpha', 0.3, 'DisplayName', areaname);
            title('FLEXION: ANT EMG - mean & std dev.')
            xlabel('Task percentage of completion')
            ylabel('EMG level [% MVC]')
            xtickformat('percentage')
            ytickformat('percentage')
            legend;
        end
    end

    % saving relevant data
    if robot_ON == 0
        tasks_idx_flexion_OFF = tasks_idx;
        new_motion_idx_flexion_OFF = new_motion_idx;
        n_tasks_flexion_OFF = n_tasks;
        idx4task_flexion_OFF = idx4task;
        emgMID_resamp_flexion_OFF = emgMID_resamp;
        emgANT_resamp_flexion_OFF = emgANT_resamp;
        elev_resamp_flexion_OFF = elev_resamp;
        kilos4task_flexion_OFF = kilos4task;
    else
        tasks_idx_flexion_ON = tasks_idx;
        new_motion_idx_flexion_ON = new_motion_idx;
        n_tasks_flexion_ON = n_tasks;
        idx4task_flexion_ON = idx4task;
        emgMID_resamp_flexion_ON = emgMID_resamp;
        emgANT_resamp_flexion_ON = emgANT_resamp;
        elev_resamp_flexion_ON = elev_resamp;
        kilos4task_flexion_ON = kilos4task;
    end
end


%% Dynamic and Static phases separation %%

zero_mat = zeros(17,6);
S1 = struct('elev_hist_abd_NO', {elev_resamp_abduction_NO}, 'elev_hist_flex_NO', {elev_resamp_flexion_NO},...
            'elev_hist_abd_ON', {elev_resamp_abduction_ON}, 'elev_hist_flex_ON', {elev_resamp_flexion_ON},...
            'elev_hist_abd_OFF', {elev_resamp_abduction_OFF}, 'elev_hist_flex_OFF', {elev_resamp_flexion_OFF},...
            'emgMID_hist_abd_NO', {emgMID_resamp_abduction_NO}, 'emgMID_hist_flex_NO', {emgMID_resamp_flexion_NO},...
            'emgMID_hist_abd_ON', {emgMID_resamp_abduction_ON}, 'emgMID_hist_flex_ON', {emgMID_resamp_flexion_ON},...
            'emgMID_hist_abd_OFF', {emgMID_resamp_abduction_OFF}, 'emgMID_hist_flex_OFF', {emgMID_resamp_flexion_OFF},...
            'emgANT_hist_abd_NO', {emgANT_resamp_abduction_NO}, 'emgANT_hist_flex_NO', {emgANT_resamp_flexion_NO},...
            'emgANT_hist_abd_ON', {emgANT_resamp_abduction_ON}, 'emgANT_hist_flex_ON', {emgANT_resamp_flexion_ON},...
            'emgANT_hist_abd_OFF', {emgANT_resamp_abduction_OFF}, 'emgANT_hist_flex_OFF', {emgANT_resamp_flexion_OFF},...
            'kilos4task_abd_NO', kilos4task_abduction_NO, 'kilos4task_flex_NO', kilos4task_flexion_NO,...
            'kilos4task_abd_ON', kilos4task_abduction_ON, 'kilos4task_flex_ON', kilos4task_flexion_ON,...
            'kilos4task_abd_OFF', kilos4task_abduction_OFF, 'kilos4task_flex_OFF', kilos4task_flexion_OFF,...
            'elev_max_ROM', zero_mat, ...
            'dynEMGup_MID_ROM', zero_mat, 'dynEMGdown_MID_ROM', zero_mat, 'statEMG_MID_ROM', zero_mat, ...
            'dynEMGup_ANT_ROM', zero_mat, 'dynEMGdown_ANT_ROM', zero_mat, 'statEMG_ANT_ROM', zero_mat, ...
            'elev_max_0kg', zero_mat, ...
            'dynEMGup_MID_0kg', zero_mat, 'dynEMGdown_MID_0kg', zero_mat, 'statEMG_MID_0kg', zero_mat, ...
            'dynEMGup_ANT_0kg', zero_mat, 'dynEMGdown_ANT_0kg', zero_mat, 'statEMG_ANT_0kg', zero_mat, ...
            'elev_max_2kg', zero_mat, ...
            'dynEMGup_MID_2kg', zero_mat, 'dynEMGdown_MID_2kg', zero_mat, 'statEMG_MID_2kg', zero_mat,...
            'dynEMGup_ANT_2kg', zero_mat, 'dynEMGdown_ANT_2kg', zero_mat, 'statEMG_ANT_2kg', zero_mat);

for ii = 1:6
    
    if ii == 1 % abd NO
        elevation_datatype = elev_resamp_abduction_NO;
        emgMID_datatype = emgMID_resamp_abduction_NO;
        emgANT_datatype = emgANT_resamp_abduction_NO;
        kilo_dataype = kilos4task_abduction_NO;
    elseif ii == 2 % flex NO
        elevation_datatype = elev_resamp_flexion_NO;
        emgMID_datatype = emgMID_resamp_flexion_NO;
        emgANT_datatype = emgANT_resamp_flexion_NO;
        kilo_dataype = kilos4task_flexion_NO;
    elseif ii == 3 % abd OFF
        elevation_datatype = elev_resamp_abduction_OFF;
        emgMID_datatype = emgMID_resamp_abduction_OFF;
        emgANT_datatype = emgANT_resamp_abduction_OFF;
        kilo_dataype = kilos4task_abduction_OFF;
    elseif ii == 4 % flex OFF
        elevation_datatype = elev_resamp_flexion_OFF;
        emgMID_datatype = emgMID_resamp_flexion_OFF;
        emgANT_datatype = emgANT_resamp_flexion_OFF;
        kilo_dataype = kilos4task_flexion_OFF;
    elseif ii == 5 % abd ON
        elevation_datatype = elev_resamp_abduction_ON;
        emgMID_datatype = emgMID_resamp_abduction_ON;
        emgANT_datatype = emgANT_resamp_abduction_ON;
        kilo_dataype = kilos4task_abduction_ON;
    else % flex ON
        elevation_datatype = elev_resamp_flexion_ON;
        emgMID_datatype = emgMID_resamp_flexion_ON;
        emgANT_datatype = emgANT_resamp_flexion_ON;
        kilo_dataype = kilos4task_flexion_ON;
    end


    for idx_data = 1:size(elevation_datatype,2)
        tmp_elev = elevation_datatype{idx_data};
        tmp_emgMID = emgMID_datatype{idx_data};
        tmp_emgANT = emgANT_datatype{idx_data};
        tmp_kilo = kilo_dataype(idx_data);
        dynamic2static = find(abs(diff(tmp_elev))<1e-2);
        dynamic2static = dynamic2static(dynamic2static>50);
        dynamic2static = dynamic2static(dynamic2static<1450);
        start_hold = min(dynamic2static);
        end_hold = max(dynamic2static);
        first_120 = find(tmp_elev(1:start_hold)<120,1,'last');
        last_120 = find(tmp_elev(end_hold:end)<120,1) + end_hold;

        % ---------- Output Data Extraction ---------- %

        % max elevation
        elev_max = max(tmp_elev);
        
        if tmp_kilo == 1
            end_integral_up = first_120;
            start_integral_down = last_120;
        else
            end_integral_up = start_hold;
            start_integral_down = end_hold;
        end

        % integral MID EMG dynamic up
        dynEMGup_MID_int = cumtrapz(tmp_emgMID(1:end_integral_up));
        dynEMGup_MID_int = dynEMGup_MID_int(end);
        % integral MID EMG dynamic down
        dynEMGdown_MID_int = cumtrapz(tmp_emgMID(start_integral_down:end));
        dynEMGdown_MID_int = dynEMGdown_MID_int(end);

        % integral ANT EMG dynamic up
        dynEMGup_ANT_int = cumtrapz(tmp_emgANT(1:end_integral_up));
        dynEMGup_ANT_int = dynEMGup_ANT_int(end);
        % integral ANT EMG dynamic down
        dynEMGdown_ANT_int = cumtrapz(tmp_emgANT(start_integral_down:end));
        dynEMGdown_ANT_int = dynEMGdown_ANT_int(end);

        % mean MID EMG static
        statEMGavg_MID = mean(tmp_emgMID(start_hold:end_hold));
        statEMGavg_MID = statEMGavg_MID(end);

        % mean ANT EMG static
        statEMGavg_ANT = mean(tmp_emgANT(start_hold:end_hold));
        statEMGavg_ANT = statEMGavg_ANT(end);
        

        % output data Struct
        if tmp_kilo == 1
            S1.elev_max_ROM(idx_data,ii) = elev_max;
            S1.dynEMGup_MID_ROM(idx_data,ii) = dynEMGup_MID_int;
            S1.dynEMGdown_MID_ROM(idx_data,ii) = dynEMGdown_MID_int;
            S1.dynEMGup_ANT_ROM(idx_data,ii) = dynEMGup_ANT_int;
            S1.dynEMGdown_ANT_ROM(idx_data,ii) = dynEMGdown_ANT_int;
            S1.statEMG_MID_ROM(idx_data,ii) = statEMGavg_MID;
            S1.statEMG_ANT_ROM(idx_data,ii) = statEMGavg_ANT;
        elseif tmp_kilo == 0
            S1.elev_max_0kg(idx_data,ii) = elev_max;
            S1.dynEMGup_MID_0kg(idx_data,ii) = dynEMGup_MID_int;
            S1.dynEMGdown_MID_0kg(idx_data,ii) = dynEMGdown_MID_int;
            S1.dynEMGup_ANT_0kg(idx_data,ii) = dynEMGup_ANT_int;
            S1.dynEMGdown_ANT_0kg(idx_data,ii) = dynEMGdown_ANT_int;
            S1.statEMG_MID_0kg(idx_data,ii) = statEMGavg_MID;
            S1.statEMG_ANT_0kg(idx_data,ii) = statEMGavg_ANT;
        else
            S1.elev_max_2kg(idx_data,ii) = elev_max;
            S1.dynEMGup_MID_2kg(idx_data,ii) = dynEMGup_MID_int;
            S1.dynEMGdown_MID_2kg(idx_data,ii) = dynEMGdown_MID_int;
            S1.dynEMGup_ANT_2kg(idx_data,ii) = dynEMGup_ANT_int;
            S1.dynEMGdown_ANT_2kg(idx_data,ii) = dynEMGdown_ANT_int;
            S1.statEMG_MID_2kg(idx_data,ii) = statEMGavg_MID;
            S1.statEMG_ANT_2kg(idx_data,ii) = statEMGavg_ANT;
        end

        % plots
        if show_separation == 1
            figure(1000)
            plot(elevation_datatype{idx_data})
            hold on
            grid on
            xline(first_120)
            xline(last_120)
            xline(start_hold)
            xline(end_hold)
            title(['TypeData = ' num2str(ii) ' | IdxData = ' num2str(idx_data)])
            hold off
            pause()
        end
    end

    % average values computation
    S1.elev_max_ROM(17,ii) = sum(S1.elev_max_ROM(:,ii))/nnz(S1.elev_max_ROM(:,ii));
    S1.dynEMGup_MID_ROM(17,ii) = sum(S1.dynEMGup_MID_ROM(:,ii))/nnz(S1.dynEMGup_MID_ROM(:,ii));
    S1.dynEMGdown_MID_ROM(17,ii) = sum(S1.dynEMGdown_MID_ROM(:,ii))/nnz(S1.dynEMGdown_MID_ROM(:,ii));
    S1.dynEMGup_ANT_ROM(17,ii) = sum(S1.dynEMGup_ANT_ROM(:,ii))/nnz(S1.dynEMGup_ANT_ROM(:,ii));
    S1.dynEMGdown_ANT_ROM(17,ii) = sum(S1.dynEMGdown_ANT_ROM(:,ii))/nnz(S1.dynEMGdown_ANT_ROM(:,ii));
    S1.statEMG_MID_ROM(17,ii) = sum(S1.statEMG_MID_ROM(:,ii))/nnz(S1.statEMG_MID_ROM(:,ii));
    S1.statEMG_ANT_ROM(17,ii) = sum(S1.statEMG_ANT_ROM(:,ii))/nnz(S1.statEMG_ANT_ROM(:,ii));
    
    S1.elev_max_0kg(17,ii) = sum(S1.elev_max_0kg(:,ii))/nnz(S1.elev_max_0kg(:,ii));
    S1.dynEMGup_MID_0kg(17,ii) = sum(S1.dynEMGup_MID_0kg(:,ii))/nnz(S1.dynEMGup_MID_0kg(:,ii));
    S1.dynEMGdown_MID_0kg(17,ii) = sum(S1.dynEMGdown_MID_0kg(:,ii))/nnz(S1.dynEMGdown_MID_0kg(:,ii));
    S1.dynEMGup_ANT_0kg(17,ii) = sum(S1.dynEMGup_ANT_0kg(:,ii))/nnz(S1.dynEMGup_ANT_0kg(:,ii));
    S1.dynEMGdown_ANT_0kg(17,ii) = sum(S1.dynEMGdown_ANT_0kg(:,ii))/nnz(S1.dynEMGdown_ANT_0kg(:,ii));
    S1.statEMG_MID_0kg(17,ii) = sum(S1.statEMG_MID_0kg(:,ii))/nnz(S1.statEMG_MID_0kg(:,ii));
    S1.statEMG_ANT_0kg(17,ii) = sum(S1.statEMG_ANT_0kg(:,ii))/nnz(S1.statEMG_ANT_0kg(:,ii));
    
    S1.elev_max_2kg(17,ii) = sum(S1.elev_max_2kg(:,ii))/nnz(S1.elev_max_2kg(:,ii));
    S1.dynEMGup_MID_2kg(17,ii) = sum(S1.dynEMGup_MID_2kg(:,ii))/nnz(S1.dynEMGup_MID_2kg(:,ii));
    S1.dynEMGdown_MID_2kg(17,ii) = sum(S1.dynEMGdown_MID_2kg(:,ii))/nnz(S1.dynEMGdown_MID_2kg(:,ii));
    S1.dynEMGup_ANT_2kg(17,ii) = sum(S1.dynEMGup_ANT_2kg(:,ii))/nnz(S1.dynEMGup_ANT_2kg(:,ii));
    S1.dynEMGdown_ANT_2kg(17,ii) = sum(S1.dynEMGdown_ANT_2kg(:,ii))/nnz(S1.dynEMGdown_ANT_2kg(:,ii));
    S1.statEMG_MID_2kg(17,ii) = sum(S1.statEMG_MID_2kg(:,ii))/nnz(S1.statEMG_MID_2kg(:,ii));
    S1.statEMG_ANT_2kg(17,ii) = sum(S1.statEMG_ANT_2kg(:,ii))/nnz(S1.statEMG_ANT_2kg(:,ii));
end

if save_data == 1
    save_path = 'C:\Users\loren\Desktop\TESTS\ICRA\Output_Data\S1_struct.mat';
    save(save_path,'-struct','S1')
end
