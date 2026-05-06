function dXdt = spacecraft_dynamics(t, X, I, rho, v_orb, Cd, geom)
    % state variables
    q = X(1:4);
    q = q / norm(q);
    w = X(5:7);

    % rotate q
    Omega = [0, -w(1), -w(2), -w(3);
             w(1),  0,   w(3), -w(2);
             w(2), -w(3), 0,    w(1);
             w(3),  w(2), -w(1), 0];
    dq = 0.5 * Omega * q;

    % rotation Matrix
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1 - 2*(qy^2 + qz^2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw);
         2*(qx*qy + qz*qw),     1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);
         2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),   1 - 2*(qx^2 + qy^2)];

    % orbital velocity direction
    v_orbital = [v_orb; 0; 0];
    v_body = R' * v_orbital;
    v_unit = v_body / norm(v_body);
    v_mag_sq = sum(v_body.^2);

    % torque and drag force
    tau_aero = [0; 0; 0];
    for i = 1:6
        n_face = geom(i, 1:3)';
        A_face = geom(i, 4);
        r_face = geom(i, 5:7)';
        
        proj = dot(n_face, v_unit);
        if proj > 0
            F_face = -0.5 * rho * v_mag_sq * Cd * (A_face * proj) * v_unit;
            tau_aero = tau_aero + cross(r_face, F_face);
        end
    end

    dw = I \ (tau_aero - cross(w, I * w));
    dXdt = [dq; dw];
end