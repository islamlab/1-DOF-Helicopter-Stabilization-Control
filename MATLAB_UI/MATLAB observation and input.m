clear; clc; close all;

%% ================= USER / SYSTEM PARAMETERS =================
theta_trim_deg = 29;      % MUST match Arduino
sendThreshold  = 0.05;    % deg
sendRate       = 0.04;    % seconds (50 Hz)

ARDUINO_MIN = 0;          % Arduino safety limits
ARDUINO_MAX = 80;

%% ================= UI =================
ui = uifigure('Name','Theta Command','Position',[100 100 320 230]);

uilabel(ui,...
    'Text','Desired Physical Angle (deg)',...
    'Position',[40 180 240 22],...
    'HorizontalAlignment','center');

thetaField = uieditfield(ui,'numeric',...
    'Value',20,...
    'Limits',[0 80],...
    'Position',[85 145 150 30]);

thetaSlider = uislider(ui,...
    'Limits',[0 80],...
    'Value',20,...
    'MajorTicks',0:10:80,...
    'Position',[40 110 240 3]);

thetaSlider.ValueChangedFcn = @(s,~) set(thetaField,'Value',s.Value);
thetaField.ValueChangedFcn  = @(f,~) set(thetaSlider,'Value',f.Value);

%% ================= SERIAL =================
ports = serialportlist("available");
if isempty(ports)
    error("No Arduino detected. Close Arduino IDE.");
end

s = serialport(ports(1),115200);
configureTerminator(s,"LF");
flush(s);

disp("Connected to Arduino.");

%% ================= DATA STORAGE =================
N = 1500;
angleData = nan(1,N);
refData   = nan(1,N);
pwmData   = nan(1,N);
k = 1;

%% ================= PLOTS =================
figure('Name','Arduino Monitoring');

subplot(2,1,1)
h1 = plot(angleData,'y','LineWidth',1.5); hold on;
h2 = plot(refData,'r--','LineWidth',1.5);
ylim([-10 80]);
ylabel('Angle (deg)');
legend('Measured','Reference','Location','best');
grid on;

subplot(2,1,2)
h3 = plot(pwmData,'y','LineWidth',1.5);
ylim([1000 1150]);
xlabel('Sample');
ylabel('PWM (\mus)');
grid on;

%% ================= MAIN LOOP =================
disp("Sending reference and monitoring Arduino output...");
lastSent = NaN;

while isvalid(ui) && k <= N

    %% ---- READ USER INPUT (PHYSICAL ANGLE) ----
    theta_physical = thetaSlider.Value;

    %% ---- CLAMP TO ARDUINO SAFETY LIMITS ----
    theta_physical_cmd = min(max(theta_physical, ARDUINO_MIN), ARDUINO_MAX);

    %% ---- CONVERT TO TRIMMED COMMAND ----
    theta_cmd = theta_physical_cmd - theta_trim_deg;

    %% ---- SEND TO ARDUINO ----
    if isnan(lastSent) || abs(theta_cmd - lastSent) > sendThreshold
        writeline(s, sprintf("%.2f", theta_cmd));
        lastSent = theta_cmd;
    end

    %% ---- READ ARDUINO OUTPUT ----
    while s.NumBytesAvailable > 0
        line = readline(s);

        if contains(line,"Angle:")
            nums = regexp(line,'[-+]?\d*\.?\d+','match');

            if numel(nums) >= 3
                angleData(k) = str2double(nums{1}) + theta_trim_deg;
                refData(k)   = str2double(nums{2}) + theta_trim_deg;
                pwmData(k)   = str2double(nums{3});

                set(h1,'YData',angleData);
                set(h2,'YData',refData);
                set(h3,'YData',pwmData);

                drawnow limitrate;
                k = k + 1;
            end
        end
    end

    pause(sendRate);
end

%% ================= CLEANUP =================
clear s
disp("Monitoring stopped. Serial port released.");
