function [Td, d_Td, varargout] = task_space_traj(ti, tf, Ti, Tf)

    
    pi = Ti(1:3,4);
    pf = Tf(1:3,4);

    Ri = Ti(1:3,1:3);
    Rf = Tf(1:3,1:3);
    
    Rif = Ri'*Rf;
    r = rotm2axang(Rif)';
    th_f = r(4);
    r(4) = [];


    [s, d_s, dd_s] = compute_quintic(ti, tf, [0,0,0]', [1,0,0]', false);

    pd = zeros(3,length(s));
    d_pd = zeros(3,length(s));
    dd_pd = zeros(3,length(s));

    th = zeros(1,length(s));
    d_th = zeros(1,length(s));
    dd_th = zeros(1,length(s));
    
    Td = zeros(4,4,length(s));
    d_Td = zeros(4,4,length(s));

    for i=1:length(s) 
        
        %Rectilinear path
        pd(:,i) = pi + (s(i))*(pf-pi);
        d_pd(:,i) = d_s(i)*(pf-pi);
        dd_pd(:,i) = dd_s(i)*(pf-pi);

        %Angular path
        th(i) = s(i)*th_f;
        d_th(i) = d_s(i)*th_f;
        dd_th(i) = dd_s(i)*th_f;

        

        Td(:,:,i) = [eye(3), pd(:,i); 0 0 0 1];
        d_Td(:,:,i) = [eye(3), d_pd(:,i); 0 0 0 1];
    
    end
    varargout{1} = dd_pd;

end