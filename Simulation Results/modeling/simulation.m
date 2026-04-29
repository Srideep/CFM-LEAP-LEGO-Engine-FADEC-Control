load("/MATLAB Drive/CFM-LEAP-LEGO-Engine-FADEC-Control/out.mat");
%% Extract all signals
tla    = logsout.getElement('TLA_deg').Values;
n1_dem = logsout.getElement('N1_demand').Values;
pid_o  = logsout.getElement('PID_output').Values;
n1_lim = logsout.getElement('PWM_cmd').Values;
n1_act = logsout.getElement('N1_actual').Values;
egt    = logsout.getElement('EGT').Values;
ps3    = logsout.getElement('Ps3').Values;
wf     = logsout.getElement('Wf').Values;
fn     = logsout.getElement('Fn').Values;
af     = logsout.getElement('accel_flag').Values;
ef     = logsout.getElement('egt_flag').Values;
of     = logsout.getElement('ovsd_flag').Values;

%% 8-panel EICAS plot
figure('Name', 'Full FADEC Loop Results', 'Position', [50 50 1200 900]);

subplot(4,2,1);
plot(n1_dem.Time, n1_dem.Data, 'b--', 'LineWidth', 1.2); hold on;
plot(n1_act.Time, n1_act.Data, 'b-', 'LineWidth', 1.5);
plot(n1_lim.Time, n1_lim.Data, 'r:', 'LineWidth', 1);
yline(104, 'r-', 'Redline'); yline(19, 'k:', 'Idle');
ylabel('N1 (%)'); title('N1 spool speed');
legend('Demand', 'Actual', 'Limited', 'Location', 'best');
grid on;

subplot(4,2,2);
plot(egt.Time, egt.Data, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5);
yline(1060, 'r-', 'EGT Redline'); yline(1030, 'r:', 'Margin');
ylabel('EGT (°C)'); title('Exhaust gas temperature');
grid on;

subplot(4,2,3);
plot(fn.Time, fn.Data, 'Color', [0.1 0.6 0.3], 'LineWidth', 1.5);
ylabel('Fn (kN)'); title('Net thrust');
grid on;

subplot(4,2,4);
plot(wf.Time, wf.Data, 'Color', [0.5 0.2 0.7], 'LineWidth', 1.5);
ylabel('Wf (kg/hr)'); title('Fuel flow');
grid on;

subplot(4,2,5);
plot(tla.Time, tla.Data, 'Color', [0.7 0.5 0], 'LineWidth', 1.5);
ylabel('TLA (deg)'); title('Throttle lever angle');
grid on;

subplot(4,2,6);
plot(pid_o.Time, pid_o.Data, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5);
hold on;
plot(n1_lim.Time, n1_lim.Data, 'r:', 'LineWidth', 1);
ylabel('Command (%)'); title('PID output vs limited');
legend('PID raw', 'After limits', 'Location', 'best');
grid on;

subplot(4,2,7);
area(af.Time, af.Data * 3, 'FaceColor', [1 0.8 0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6); hold on;
area(ef.Time, ef.Data * 2, 'FaceColor', [1 0.4 0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6);
area(of.Time, of.Data * 1, 'FaceColor', [1 0 0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6);
ylabel('Protection'); title('Limit flags');
yticks([0 1 2 3]); yticklabels({'Off', 'Overspd', 'EGT', 'Accel'});
legend('Accel', 'EGT', 'Overspeed'); grid on;

subplot(4,2,8);
plot(ps3.Time, ps3.Data, 'Color', [0.3 0.6 0.6], 'LineWidth', 1.5);
ylabel('Ps3 (psia)'); title('Compressor discharge pressure');
xlabel('Time (s)'); grid on;

sgtitle('Complete FADEC closed-loop simulation');

%% Save
save('fadec_full_loop_results.mat', 'logsout');
fprintf('\nSaved to fadec_full_loop_results.mat\n');

%% Print summary
fprintf('\n=== FADEC Full Loop Summary ===\n');
fprintf('N1 max:     %.1f %%\n', max(n1_act.Data));
fprintf('EGT max:    %.0f °C\n', max(egt.Data));
fprintf('Thrust max: %.1f kN\n', max(fn.Data));
fprintf('Wf max:     %.0f kg/hr\n', max(wf.Data));
fprintf('Accel lim:  active %.1f%% of time\n', 100*mean(af.Data));
fprintf('EGT lim:    active %.1f%% of time\n', 100*mean(ef.Data));
fprintf('Ovspd lim:  active %.1f%% of time\n', 100*mean(of.Data));
fprintf('===============================\n');
clear;