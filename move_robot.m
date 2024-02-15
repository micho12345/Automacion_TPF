function q = move_robot(Robot, x0, xf, R, q0)
    T0 = [R' -R' * x0'; 0 0 0 1];
    T0 = inv(T0);

    Tf = [R' -R' * xf'; 0 0 0 1];
    Tf = inv(Tf);

    N = 15;
    Ttraj = ctraj(T0, Tf, N);
    q = Robot.ikine(Ttraj, 'q0', q0, 'mask', [1 1 1 0 0 1], 'tol', 0.1);
end