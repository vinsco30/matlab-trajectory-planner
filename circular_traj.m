clear
close all

% Position waypoints definition
p0 = [0 0 0]';
c1 = [0.5 0.5 -1]'; %Point 1 of the circle
c2 = [0.75 0.75 -1]'; %Point 2 of the circle
c3 = [0.5 1 -1]'; %Point 3 of the circle
p4 = [0.5 0.5 -1]';
p5 = [0 0 0]';
% Orientation waypoints definition
R0 = eye(3);
R1 = eye(3);
R2 = eye(3);
R3 = rotx(pi/6);
R4 = eye(3);
R5 = eye(3);

p_des = [p0 c1 c2 c3 p4 p5];
R_des = [R0 R1 R2 R3 R4 R5];
t_seg = [2 2];
t_c = 15;

t_tot = sum(t_seg);
n = (t_tot+t_c)*100;

T1 = [R0, p_des(:,1); 0 0 0 1];
T2 = [R1, p_des(:,2); 0 0 0 1];
T3 = [R2, p_des(:,3); 0 0 0 1];
T4 = [R3, p_des(:,4); 0 0 0 1];
T5 = [R4, p_des(:,5); 0 0 0 1];
T6 = [R5, p_des(:,6); 0 0 0 1];

[Tl1, d_Tl1, dd_p1, omega1, d_omega1] = task_space_traj(0, t_seg(1), T1, T2);
[Tc, d_Tc, dd_pc,omegac, d_omegac] = task_space_circular(0,t_c, T2, T3, T4);
[Tl2,d_Tl2, dd_p2, omega2, d_omega2] = task_space_traj(0, t_seg(2), T5, T6);

T_step1 = cat(3, Tl1, Tc);
T_tot = cat(3, T_step1, Tl2);

d_T_step1 = cat(3, d_Tl1, d_Tc);
d_T_tot = cat(3, d_T_step1, d_Tl2);

dd_p = [dd_p1 dd_pc dd_p2];
we = [omega1 omegac omega2];
d_we = [d_omega1 d_omegac d_omega2];

% Cut the singleton dimension
x = squeeze(T_tot(1,4,:));
y = squeeze(T_tot(2,4,:));
z = squeeze(T_tot(3,4,:));
vx = squeeze(d_T_tot(1,4,:));
vy = squeeze(d_T_tot(2,4,:));
vz = squeeze(d_T_tot(3,4,:));

Rf = T_tot(1:3,1:3,:);
eul = zeros(3,n+50);

for i=1:n
    eul(:,i) = rotm2eul(Rf(:,:,i));
end

%% Plot
figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
subplot(3,1,1)
plot(x,'Color','[0.7,0.7,0.7]',LineWidth=3);
title('Task Space Trajectory','fontsize',20, 'interpreter','latex')
ylabel('$p_d$ $[m]$','fontsize',20, 'interpreter','latex')
grid on
hold on
plot(y,'--','Color','[0.7,0.7,0.7]',LineWidth=3);
plot(z,'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:500:n+50;
ax.XTickLabel = {'0', '5', '10', '20', '30'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
legend({'$p_x$', '$p_y$', '$p_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
subplot(3,1,2)
plot(vx,'Color','[0.7,0.7,0.7]',LineWidth=3);
ylabel('$\dot{p}_d$ $[m/s]$','fontsize',20, 'interpreter','latex')
grid on
hold on
plot(vy,'--','Color','[0.7,0.7,0.7]',LineWidth=3);
plot(vz,'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:500:n+50;
ax.XTickLabel = {'0', '5', '10', '20', '30'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
legend({'$v_x$', '$v_y$', '$v_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
subplot(3,1,3)
plot(dd_p(1,:),'Color','[0.7,0.7,0.7]',LineWidth=3);
ylabel('$\ddot{p}_d$ $[m/s^2]$','fontsize',20, 'interpreter','latex')
grid on
hold on
plot(dd_p(2,:),'--','Color','[0.7,0.7,0.7]',LineWidth=3);
plot(dd_p(3,:),'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:500:n+50;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '5', '10', '15', '20'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
legend({'$a_x$', '$a_y$', '$a_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');

%% Plot orientations
figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
subplot(3,1,1)
plot(eul(3,:),'Color','[0.7,0.7,0.7]',LineWidth=3);
title('Task Space Orientation','fontsize',20, 'interpreter','latex')
ylabel('$\phi_d$ $[m]$','fontsize',20, 'interpreter','latex')
grid on
hold on
plot(eul(2,:),'--','Color','[0.7,0.7,0.7]',LineWidth=3);
plot(eul(1,:),'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:500:n+50;
ax.XTickLabel = {'0', '5', '10', '15', '20', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
legend({'$\phi_x$', '$\phi_y$', '$\phi_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
subplot(3,1,2)
plot(we(1,:),'Color','[0.7,0.7,0.7]',LineWidth=3);
ylabel('$\omega_d$ $[rad/s]$','fontsize',20, 'interpreter','latex')
grid on
hold on
plot(we(2,:),'--','Color','[0.7,0.7,0.7]',LineWidth=3);
plot(we(3,:),'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:500:n+50;
ax.XTickLabel = {'0', '5', '10', '15', '20', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
legend({'$\omega_x$', '$\omega_y$', '$\omega_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
subplot(3,1,3)
plot(d_we(1,:),'Color','[0.7,0.7,0.7]',LineWidth=3);
ylabel('$\dot{\omega}_d$ $[rad/s^2]$','fontsize',20, 'interpreter','latex')
grid on
hold on
plot(d_we(2,:),'--','Color','[0.7,0.7,0.7]',LineWidth=3);
plot(d_we(3,:),'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:500:n+50;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '5', '10', '15', '20', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
legend({'$\dot{\omega}_x$', '$\dot{\omega}_y$', '$\dot{\omega}_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');

%% Plot 3D
figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
plot3(x,y,z,'Color','[0.0,0.0,0.0]',LineWidth=3)
title('3D Task Space Trajectory','fontsize',20, 'interpreter','latex')
grid on
xlabel('$x$ $[m]$','fontsize',20,'interpreter','latex')
ylabel('$y$ $[m]$','fontsize',20, 'interpreter','latex')
zlabel('$z$ $[m]$','fontsize',20, 'interpreter','latex')








