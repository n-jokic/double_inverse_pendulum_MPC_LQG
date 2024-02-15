function plot_simulation_result(states, ...
    states_measured, control, state_reference, control_reference, ...
    disturbance, t, u_limit)

radian_to_degrees= 1/pi*180;
k = 1;

figure(1);
subplot(5, 1, k);
hold on;
plot(t, states_measured(1:end-1, k)*radian_to_degrees, '.', ...
    'MarkerSize', 2);
plot(t, states(1:end-1, k)*radian_to_degrees);
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('\beta [deg]');
xlabel('t [s]');
grid on;
legend(['x_' num2str(k) '(t)' '+ w(t)'], ['x_' num2str(k) '(t)'], ...
    ['x_{' num2str(k) ', ref}']);

k = 2;

subplot(5, 1, k);
hold on;
plot(t, states_measured(1:end-1, k)*radian_to_degrees, '.', ...
    'MarkerSize', 2);
plot(t, states(1:end-1, k)*radian_to_degrees);
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('a_{\beta} [deg/s]');
xlabel('t [s]');
grid on;
legend(['x_' num2str(k) '(t)' '+ w(t)'], ['x_' num2str(k) '(t)'], ...
    ['x_{' num2str(k) ', ref}']);

k = 3;

subplot(5, 1, k);
hold on;
plot(t, states_measured(1:end-1, k)*radian_to_degrees, '.', ...
    'MarkerSize', 2);
plot(t, states(1:end-1, k)*radian_to_degrees);
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('\alpha [deg]');
xlabel('t [s]');
grid on;
legend(['x_' num2str(k) '(t)' '+ w(t)'], ['x_' num2str(k) '(t)'], ...
    ['x_{' num2str(k) ', ref}']);

k = 4;
subplot(5, 1, k);
hold on;
plot(t, states_measured(1:end-1, k)*radian_to_degrees, '.', ...
    'MarkerSize', 2);
plot(t, states(1:end-1, k)*radian_to_degrees);
plot(t, state_reference(1:end, k)*radian_to_degrees, 'k--');
ylabel('a_{\alpha} [deg/s]');
xlabel('t [s]');
grid on;
legend(['x_' num2str(k) '(t)' '+ w(t)'], ['x_' num2str(k) '(t)'], ...
    ['x_{' num2str(k) ', ref}']);

k = 5;
subplot(5, 1, k);
hold on;
plot(t, states_measured(1:end-1, k), '.', ...
    'MarkerSize', 2);
plot(t, states(1:end-1, k));
plot(t, state_reference(1:end, k), 'k--');
ylabel('i [A]');
xlabel('t [s]');
grid on;
legend(['x_' num2str(k) '(t)' '+ w(t)'], ['x_' num2str(k) '(t)'], ...
    ['x_{' num2str(k) ', ref}']);


figure(2);
hold on;
stairs(t, control);
stairs(t, control_reference);
hold on;
%stairs(t, disturbance, 'k--');
plot(t, u_limit(1)*ones(size(t)), 'r--');
plot(t, u_limit(2)*ones(size(t)), 'r--');
xlabel('t [s]');
ylabel('u [V]');

legend('u [V]', 'u_{ref} [V]');
hold off;


end

