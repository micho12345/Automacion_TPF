% drawLine: Controla al robot para que dibuje la linea con el end effector.
function drawLine(Robot, r0, rf, table_start, table_end, z_offset, q0)
    x0 = table_start + r0;
    xf = table_start + rf;
    x0_offset = x0 + [0, 0, z_offset];
    xf_offset = xf + [0, 0, z_offset];

    R = [0,0,1; 0,1,0; -1,0,0];
    q1 = move_robot(Robot, x0_offset, x0, R, q0);
    q2 = move_robot(Robot, x0, xf, R, q0);
    q3 = move_robot(Robot, xf, xf_offset, R, q0);

    % GRÁFICOS
    % Rectángulo
    drawRectangle(table_start, table_end)

    % Robot
    Robot.plot(q1)
    Robot.plot(q2)
    Robot.plot(q3)
    hold on

    % Línea
    xline = linspace(x0(1), xf(1), 100);
    yline = linspace(x0(2), xf(2), 100);
    zline = linspace(x0(3), xf(3), 100);
    plot3(xline, yline, zline, 'LineWidth', 2, 'Color', 'r')
    hold off
end

function drawRectangle(table_start, table_end)
    xRect = [table_start(1), table_end(1), table_end(1), table_start(1)];
    yRect = [table_start(2), table_start(2), table_end(2), table_end(2)];
    zRect = [table_start(3), table_start(3), table_start(3), table_start(3)];
    fill3(xRect, yRect, zRect, 'g'); % color azul
end
