function [xvec, yvec] = minimumJerkPlotter(q0, q1)
% generate and plot trajectory based on three states
% input: Initial state q0 = [t0, x0, y0, dx0, dy0],
%        first state q1  = [t1, x1, y1, dx1, dy1];
% output: a 6-D vector of coefficients a
xvec = zeros(1,6);
yvec = zeros(1,6);
T = q1(1)-q0(1);
x0 = q0(2);
dx0 = q0(4);
xT = q1(2);
dxT = q1(4);
xvec = minimumJerk(x0,dx0,0,xT,dxT,0,T);

y0 = q0(3);
dy0 = q0(5);
yT = q1(3);
dyT = q1(5);
yvec = minimumJerk(y0,dy0,0,yT,dyT,0,T);

t = 0:0.01:T;
trajX = xvec(1) + xvec(2)*t + xvec(3)*(t.^2) + xvec(4)*(t.^3) + xvec(5)*(t.^4) + xvec(6)*(t.^5);
trajY = yvec(1) + yvec(2)*t + yvec(3)*(t.^2) + yvec(4)*(t.^3) + yvec(5)*(t.^4) + yvec(6)*(t.^5);
plot(trajX,trajY, 'LineWidth', 3, 'color', 'green');
hold on;

end