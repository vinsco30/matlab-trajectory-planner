function [Td, d_Td, varargout] = task_space_traj(ti, tf, Ti, Tf)
    % Inputs:
    % ti - Trajectory starting time (scalar)
    % tf - Trajecotry final time (scalar)
    % Ti - Transformation matrix of the initial pose 
    % Tf - Transformation matrix of the final ee pose
    %
    % Outputs:
    % Td - vector of transformation matrices through the whole trajectory
    % d_Td = Time derivative of the previous vector
    % - varargout{1} = The vector of linear accelerations through the whole
    %                  trajectory
    % - varargout{2} = The vector of angular velocities
    % - varargout{3} = The vector of angular accelerations through the whole
    %                  trajectory


    format long
    %Computation of the initial quantities
    pi = Ti(1:3,4);
    pf = Tf(1:3,4);

    Ri = Ti(1:3,1:3);
    Rf = Tf(1:3,1:3);
    
    Rif = Ri'*Rf;
    r = rotm2axang(Rif)';
    th_f = r(4);
    r(4) = [];


    [s, d_s, dd_s] = compute_quintic(ti, tf, [0,0,0]', [norm(pf-pi,2),0,0]', false);
    [th, d_th, dd_th] = compute_quintic(ti,tf,[0,0,0]', [th_f, 0, 0]', false);

    pd = zeros(3,length(s));
    d_pd = zeros(3,length(s));
    dd_pd = zeros(3,length(s));

    % th = zeros(1,length(s));
    % d_th = zeros(1,length(s));
    % dd_th = zeros(1,length(s));

    Re = zeros(3,3,length(s));
    d_Re = zeros(3,3,length(s));
    we = zeros(3,length(s));
    d_we = zeros(3,length(s));
    
    Td = zeros(4,4,length(s));
    d_Td = zeros(4,4,length(s));

    for i=1:length(s) 
        
        %Rectilinear path
        pd(:,i) = pi + (s(i))/norm(pf-pi,2)*(pf-pi);
        d_pd(:,i) = d_s(i)/norm(pf-pi,2)*(pf-pi);
        dd_pd(:,i) = dd_s(i)/norm(pf-pi,2)*(pf-pi);
        
        % Angular velocity and acceleration of the "middle frame" R^i
        wi = d_th(i)*r;
        d_wi = dd_th(i)*r;

        % Angular path
        Re(:,:,i) = Ri*axang2rotm([r; th(i)]');
        we(:,i) = Ri*wi;
        d_we(:,i) = Ri*d_wi;
        % Te_dot and Te
        d_Re(:,:,i) = skew_symmetric(we(:,i))*Re(:,:,i);

        Td(:,:,i) = [Re(:,:,i), pd(:,i); 0 0 0 1];
        d_Td(:,:,i) = [d_Re(:,:,i), d_pd(:,i); 0 0 0 1];
    
    end
    varargout{1} = dd_pd;
    varargout{2} = we;
    varargout{3} = d_we;

end