function [Td, d_Td, varargout] = task_space_circular(ti, tf, T1, T2, T3)
    % Inputs:
    % ti - Trajectory starting time (scalar)
    % tf - Trajecotry final time (scalar)
    % T1 - Transformation matrix of first point for the circumference 
    % T2 - Transformation matrix of second point for the circumference
    % T3 - Transformation matrix of third point for the circumference
    %
    % Outputs:
    % Td - vector of transformation matrices through the whole trajectory
    % d_Td = Time derivative of the previous vector
    % - varargout{1} = The vector of linear accelerations through the whole
    %                  trajectory
    % - varargout{2} = The vector of angular velocities
    % - varargout{3} = The vector of angular accelerations through the whole
    %                  trajectory
    p1 = T1(1:3,4); %Point 1 of the circle
    p2 = T2(1:3,4); %Point 2 of the circle
    p3 = T3(1:3,4); %Point 3 of the circle

    Ri = T1(1:3,1:3);
    Rf = T3(1:3,1:3);

    Rif = Ri'*Rf;
    r = rotm2axang(Rif)';
    th_f = r(4);
    r(4) = [];

    %% Circle coefficient computation: x^2 + y^2 + ax + bx + c = 0 ==> k = M^-1 *d
    M = [p1(1), p1(2), 1;
        p2(1), p2(2), 1;
        p3(1), p3(2), 1];
    
    d = -[p1(1)^2+p1(2)^2;
        p2(1)^2+p2(2)^2;
        p3(1)^2+p3(2)^2];
    
    k = M\d;

    % Radius, center and phase
    C = [-k(1)/2; -k(2)/2; -1];
    ro = sqrt(k(1)^2/4 + k(2)^2/4 - k(3));
    ph = atan2(p1(2)-C(2),p1(1)-C(1));

    [s, d_s, dd_s] = compute_quintic(ti, tf, [0,0,0]', [2*pi*ro,0,0]', false);
    [th, d_th, dd_th] = compute_quintic(ti, tf, [0,0,0]', [th_f,0,0]', false);

    pd = zeros(3,length(s));
    d_pd = zeros(3,length(s));
    dd_pd = zeros(3,length(s));

    Re = zeros(3,3,length(s));
    d_Re = zeros(3,3,length(s));
    we = zeros(3,length(s));
    d_we = zeros(3,length(s));
    
    Td = zeros(4,4,length(s));
    d_Td = zeros(4,4,length(s));

    for i=1:length(s)
        
        pd(:,i) = [C(1) + ro*cos(s(i)/ro + ph);
                   C(2) + ro*sin(s(i)/ro + ph);
                   C(3)];
        d_pd(:,i) = [-d_s(i)*sin(s(i)/ro); 
                    d_s(i)*cos(s(i)/ro);
                    0];
        dd_pd(:,i) = [(-d_s(i)^2*cos(s(i)/ro))/ro - dd_s(i)*sin(s(i)/ro);
                       (-d_s(i)^2*sin(s(i)/ro))/ro + dd_s(i)*cos(s(i)/ro);
                       0];

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