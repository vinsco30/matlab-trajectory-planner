clear
close all

% Waypoints definition
p0 = [0 0 0]';
p1 = [1 1 -1]';
p2 = [1 2 -1]';
p3 = [2 2 -1]';
p4 = [2 1 -1]';
p5 = [1 1 -1]';
p6 = [0 0 0]';

R0 = eye(3); %TODO: orientation definition

p_des = [p0 p1 p2 p3 p4 p5 p6];
t_seg = [2 0.5 0.5 0.5 0.5 2];

t_tot = sum(t_seg);
n = t_tot*100;
T_tot = zeros(4,4,n+50);
d_T_tot = zeros(4,4,n+50);
dd_p = zeros(3,n+50);
prev_dim = 1;

%% Trajectory computation
for i=1:length(p_des)-1

    Ti = [R0, p_des(:,i); 0 0 0 1];
    Tf = [R0, p_des(:,i+1); 0 0 0 1];
    [Td, d_Td, acc] = task_space_traj(0, t_seg(i), Ti, Tf);

    if i == 1
        T_tot(:,:,i:size(Td,3)) = Td;
        d_T_tot(:,:,i:size(Td,3)) = d_Td;
        dd_p(:,i:size(Td,3)) = acc;
        prev_dim = size(Td,3);
    else
        T_tot(:,:,prev_dim:prev_dim+size(Td,3)-1) = Td;
        d_T_tot(:,:,prev_dim:prev_dim+size(Td,3)-1) = d_Td;
        dd_p(:,prev_dim:prev_dim+size(Td,3)-1) = acc;
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
plot(dd_p(3,:),'Color','[0.0,0.0,0.0]',LineWidth=3);
xlim([0 n+50]);
ax = gca;
ax.XTick = 0:100:n+50;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
legend({'$a_x$', '$a_y$', '$a_z$'}, 'NumColumns',3,'FontSize',15,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');

figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
plot3(x,y,z,'Color','[0.0,0.0,0.0]',LineWidth=3)
title('3D Task Space Trajectory','fontsize',20, 'interpreter','latex')
grid on
xlabel('$x$ $[m]$','fontsize',20,'interpreter','latex')
ylabel('$y$ $[m]$','fontsize',20, 'interpreter','latex')
zlabel('$z$ $[m]$','fontsize',20, 'interpreter','latex')


