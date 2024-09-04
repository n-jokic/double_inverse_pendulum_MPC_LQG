function [fig1, fig2] = plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    t, t1, t2, u_limit)

radian_to_degrees= 1/pi*180;
k = 1;

idx_t1 = find(t >= t1, 1);
idx_t2 = find(t <= t2, 1, 'last');

% Prikazujemo samo podatke unutar odabranog vremenskog intervala
t = t(idx_t1:idx_t2);
states = states(idx_t1:idx_t2, :);

control = control(idx_t1:idx_t2, :);
state_reference = state_reference(idx_t1:idx_t2, :);
control_reference = control_reference(idx_t1:idx_t2, :);

fig1 = figure();
fig1.Name = 'States';
subplot(5, 1, k);
hold on;
plot(t, states(1:end, k)*radian_to_degrees, 'r');
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('\alpha [deg]');
xlabel('t [s]');
grid on;



k = 2;

subplot(5, 1, k);
hold on;
plot(t, states(1:end, k)*radian_to_degrees, 'r');
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('a_{\alpha} [deg/s]');
xlabel('t [s]');
grid on;


k = 3;

subplot(5, 1, k);
hold on;
plot(t, states(1:end, k)*radian_to_degrees, 'r');
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('\beta [deg]');
xlabel('t [s]');
grid on;



k = 4;
subplot(5, 1, k);
hold on;
plot(t, states(1:end, k)*radian_to_degrees, 'r');
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('a_{\beta} [deg/s]');
xlabel('t [s]');
grid on;




k = 5;
subplot(5, 1, k);
hold on;
plot(t, states(1:end, k), 'r');
plot(t, state_reference(1:end, k), 'k--');
ylabel('i [A]');
xlabel('t [s]');
grid on;


fig2 = figure();
fig2.Name = 'Control';
hold on;
stairs(t, control, 'LineWidth', 0.8, 'Color','r');
stairs(t, control_reference, 'k--');
hold on;
%stairs(t, disturbance, 'k--');
plot(t, u_limit(1)*ones(size(t)), 'g--');
plot(t, u_limit(2)*ones(size(t)), 'g--');
xlabel('t [s]');
ylabel('u [V]');

lgd=legend('u [V]', 'u_{ref} [V]');
set(lgd,'Location','best');

hold off;


end

