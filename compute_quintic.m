function [qd, d_qd, dd_qd] = compute_quintic(ti, tf, qi, qf)
    % Inputs:
    % ti - Trajectory starting time (scalar)
    % tf - Trajecotry final time (scalar)
    % qi - Starting configuration [pos, vel, acc] (R^3 column vector)
    % qf - Final configurations [pos, vel, acc] (R^3 column vector)
    %
    % Outputs:
    % qd, d_qd, dd_qd - Planned trajectory for position, velocity and
    % acceleration computed with quitic polynomial


    %% Coefficients computation
    
    A = [1 ti ti^2 ti^3 ti^4 ti^5; 
        0 1 2*ti 3*ti^2 4*ti^3 5*ti^4;
        0 0 2 6*ti 12*ti^2 20*ti^4;
        1 tf tf^2 tf^3 tf^4 tf^5;
        0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
        0 0 2 6*tf 12*tf^2 20*tf^3];

    b = [qi; qf];

    coeff = A\b;

    %% Trajectory computation with quintic polynomial

    t = ti:0.01:tf-0.01;
    qd = zeros(length(t),1);
    d_qd = zeros(length(t),1);
    dd_qd = zeros(length(t),1);


    for i = 1:length(t)
        qd(i) = coeff(1) + coeff(2)*t(i) + coeff(3)*t(i)^2 + coeff(4)*t(i)^3 + coeff(5)*t(i)^4 + coeff(6)*t(i)^5;
        d_qd(i) = coeff(2) + 2*coeff(3)*t(i) + 3*coeff(4)*t(i)^2 + 4*coeff(5)*t(i)^3 + 5*coeff(6)*t(i)^4;
        dd_qd(i) = 2*coeff(3) + 6*coeff(4)*t(i) + 12*coeff(5)*t(i)^2 + 20*coeff(6)*t(i)^3;
    end

    %% Plots
    figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
    subplot(3,1,1)
    plot(qd,'Color','[0,0.0,0.0]',LineWidth=3);
    title('Quintic Polynomial Trajectory','fontsize',20, 'interpreter','latex')
    ylabel('$q_d$ $[rad]$','fontsize',20, 'interpreter','latex')
    grid on
    xlim([0 length(t)+50]);
    ax = gca;
    ax.XTick = 0:100:length(t)+50;
    ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
    subplot(3,1,2)
    plot(d_qd,'Color','[0,0,0]',LineWidth=3);
    ylabel('$\dot{q}_d$ $[rad/s]$','fontsize',20, 'interpreter','latex')
    grid on
    xlim([0 length(t)+50]);
    ax = gca;
    ax.XTick = 0:100:length(t)+50;
    ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
    set(gca, 'TickLabelInterpreter', 'latex');
    subplot(3,1,3)
    plot(dd_qd,'Color','[0,0,0]',LineWidth=3);
    ylabel('$\ddot{q}_d$ $[rad/s^2]$','fontsize',20, 'interpreter','latex')
    grid on
    xlim([0 length(t)+50]);
    ax = gca;
    ax.XTick = 0:100:length(t)+50;
    xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
    ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
    set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
    set(gca, 'TickLabelInterpreter', 'latex');


end