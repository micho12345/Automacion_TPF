function [xw, yw, zw] = workspace(L, Lee, thlim)

    th1 = linspace(-double(thlim(1)),double(thlim(1)),90.0)*pi/180;
    th2 = linspace(-double(thlim(2)),double(thlim(2)),10.0)*pi/180;
    th3 = linspace(-double(thlim(3)),double(thlim(3)),10.0)*pi/180;
    th4 = linspace(-double(thlim(4)),double(thlim(4)),40.0)*pi/180;  
    [o1, o2, o3, o4] = ndgrid(th1,th2,th3,th4);

    L23 = sqrt(L(2)^2 + L(3)^2);
    xw = L(5)*((0.5*cos(o1 + o2 + o3 + o4)) + (0.5*cos(o2 - o1 + o3 + o4))) + L(4)*((0.5*cos(o1 + o2 + o3)) + (0.5*cos(o2 - o1 + o3))) + L23*((0.5*cos(o1 - o2)) + (0.5*cos(o1 + o2)));
    yw = L(4)*((0.5*sin(o1 + o2 + o3)) - (0.5*sin(o2 - o1 + o3))) - L(5)*(0.5*sin(o2 - o1 + o3 + o4) - 0.5*sin(o1 + o2 + o3 + o4)) + L23*((0.5*sin(o1 - o2)) + (0.5*sin(o1 + o2)));
    zw = L(1) + L(4)*sin(o2 + o3) + L23*sin(o2) + L(5)*sin(o2 + o3 + o4);

end