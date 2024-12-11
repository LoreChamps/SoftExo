close all;
clear;
clc;

% -------------------- MANUAL VALUES --------------------  %
subject = 'S1';
end_abduction_index_MVC = 3000; % index when the test goes abd to flex
EMG_abd_threshold = 250;
EMG_flex_threshold = 250;
check_plot = 1;

%% Import data from text file

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

figure(1)
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

%% Maximum values

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
    figure(2)
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
end