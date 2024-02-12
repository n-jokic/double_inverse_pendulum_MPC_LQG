function plot_simulation_result(x, u, d, t, u_limit)

figure;
subplot(2, 1, 1);
plot(t, x(1:end-1, 1)/pi*180)
ylabel('\alpha [deg]');
xlabel('t [s]');

subplot(2, 1, 2);
plot(t, x(1:end-1, 3)/pi*180,t,0*ones(size(t)))
xlabel('t [s]');
ylabel('\beta [deg]');

figure;
stairs(t, u);
hold on;
stairs(t, d, 'k--');
plot(t, u_limit(1)*ones(size(t)), 'r--');
plot(t, u_limit(2)*ones(size(t)), 'r--');
xlabel('t [s]');
ylabel('u [V]');
hold off;


end

