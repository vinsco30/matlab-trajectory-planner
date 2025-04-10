function [Td, d_Td, varargout] = task_space_circular(ti, tf, T1, T2, T3)

    p1 = T1(1:3,4); %Point 1 of the circle
    p2 = T2(1:3,4); %Point 2 of the circle
    p3 = T3(1:3,4); %Point 3 of the circle

    R1 = T1(1:3,1:3);
    R2 = T2(1:3,1:3);
    R3 = T3(1:3,1:3);

    R12 = R1'*R2;
    R23 = R2'*R3;
    r12 = rotm2axang(R12)';
    r23 = rotm2axang(R23)';
    th_f12 = r12(4);
    th_f23 = r23(4);
    r12(4) = [];
    r23(4) = [];

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


    [s, d_s, dd_s] = compute_quintic(ti, tf, [0,0,0]', [2*pi,0,0]', false);

    pd = zeros(3,length(s));
    d_pd = zeros(3,length(s));
    dd_pd = zeros(3,length(s));
    
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

        Td(:,:,i) = [eye(3), pd(:,i); 0 0 0 1];
        d_Td(:,:,i) = [eye(3), d_pd(:,i); 0 0 0 1];

    end

    varargout{1} = dd_pd;
    
end