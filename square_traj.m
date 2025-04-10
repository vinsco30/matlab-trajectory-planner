%% Script for Square Trajectory
% With this script is possible to generate a trajectory for an end-effector
% manipulator which starts from a home position p0, and then make a square
% starting from point p1 -> p2 -> p3 -> p4 -> p5 to go back to home
% position p6=p0. During the square, the orientation is changed from 
% 0 -> pi and back to 0 about the z-axis. To change the dimension or the
% shape of the figure, modify the values of the waypoints

clear
close all

% Position waypoints definition
p0 = [0 0 0]';
p1 = [1 1 -1]';
p2 = [1 2 -1]';
p3 = [2 2 -1]';
p4 = [2 1 -1]';
p5 = [1 1 -1]';
p6 = [0 0 0]';
% Orientation waypoints definition
R0 = eye(3); 
R1 = eye(3);
R2 = rotz(pi/2);
R3 = rotz(pi);
R4 = rotz(pi/2);
R5 = rotz(0);
R6 = eye(3);

p_des = [p0 p1 p2 p3 p4 p5 p6];
R_des = [R0 R1 R2 R3 R4 R5 R6];
t_seg = [2 0.5 0.5 0.5 0.5 2];

t_tot = sum(t_seg);
n = t_tot*100;
T_tot = zeros(4,4,n+50);
d_T_tot = zeros(4,4,n+50);
dd_p = zeros(3,n+50);
we = zeros(3,n+50);
d_we = zeros(3,n+50);
prev_dim = 1;

%% Trajectory computation
for i=1:length(p_des)-1

    Ti = [R_des(:,i+2*(i-1):i+2*(i-1)+2), p_des(:,i); 0 0 0 1];
    Tf = [R_des(:,i+2*(i-1)+3:i+2*(i-1)+5), p_des(:,i+1); 0 0 0 1];
    [Td, d_Td, acc, omega, d_omega] = task_space_traj(0, t_seg(i), Ti, Tf);

    if i == 1
        T_tot(:,:,i:size(Td,3)) = Td;
        d_T_tot(:,:,i:size(Td,3)) = d_Td;
        dd_p(:,i:size(Td,3)) = acc;
        we(:,i:size(Td,3)) = omega;
        d_we(:,i:size(Td,3)) = d_omega;
        prev_dim = size(Td,3);
    else
        T_tot(:,:,prev_dim:prev_dim+size(Td,3)-1) = Td;
        d_T_tot(:,:,prev_dim:prev_dim+size(Td,3)-1) = d_Td;
        dd_p(:,prev_dim:prev_dim+size(Td,3)-1) = acc;
        we(:,prev_dim:prev_dim+size(Td,3)-1) = omega;
        d_we(:,prev_dim:prev_dim+size(Td,3)-1) = d_omega;
        prev_dim = prev_dim+size(Td,3);
    end

end

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

    
%% Plot positions
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
ax.XTick = 0:100:n+50;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
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
ax.XTick = 0:100:n+50;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
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
plot(dd_p(3,:),'Color','[0.0,0.0,0.0]',LineWidthor=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:100:n+50;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
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
ax.XTick = 0:100:n+50;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
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
ax.XTick = 0:100:n+50;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
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
ax.XTick = 0:100:n+50;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
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


