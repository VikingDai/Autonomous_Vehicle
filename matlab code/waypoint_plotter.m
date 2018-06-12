clear all;
%% waypoint processing

filename = 'loopRecord_1.csv';
M = csvread(filename, 0, 0);

latOffset = M(1,2);
longOffset = M(1,3);
[x_datum, y_datum] = deg2utm(latOffset, longOffset);

T = M(:,1);
ySet = [];
xSet = [];
xVelSet = [];
yVelSet = [];

for i = 1:size(M,1)
    [x_utm, y_utm] = deg2utm(M(i,2), M(i,3));
    y = y_utm - y_datum;
    x = x_utm - x_datum;
    linVel = M(i,4);
    theta = M(i,5);
    xVel = linVel * cos(theta);
    yVel = linVel * sin(theta);
    
    ySet = [ySet; y];
    xSet = [xSet; x];
    xVelSet = [xVelSet; xVel];
    yVelSet = [yVelSet; yVel];
end
state = [T, xSet, ySet, xVelSet, yVelSet];
csvwrite('coarseState_new.csv',state);

% figure;
% plot(xSet, ySet);
% hold on;

%% trajectory generation
recur = size(T,1)-1;

% collect the useful data here.
% delete the last state set.
aVecSet = zeros(6,recur);
bVecSet = zeros(6,recur);

for i = 1: recur
    % plot the speed vector;
    p0 = [xSet(i), ySet(i)];
    p1 = [xSet(i)+xVelSet(i), ySet(i)+yVelSet(i)];
    vectarrow(p0,p1);
    hold on;
    
    % plot the trajectory
    q0 = [T(i), xSet(i), ySet(i), xVelSet(i), yVelSet(i)];
    q1 = [T(i+1), xSet(i+1), ySet(i+1), xVelSet(i+1), yVelSet(i+1)];
    [a, b] = minimumJerkPlotter(q0, q1);
    aVecSet(:,i) = transpose(a);
    bVecSet(:,i) = transpose(b);
    msg = ['generating ',num2str(i),' out of ', num2str(recur), ' trajectories'];
    disp(msg);
end

%% Controller design
% define the state machine. Starts from stage one.
stage = 1;

% evaluate the desired state.
dt=0.01;
tsteps=T(1):dt:T(end);
N=size(tsteps, 2);
X = zeros(3,N);
U = zeros(2,N);

% with some initial error
% with no-initial error 
X(:,1)=[-2, 2, -pi/4];
U(:,1)=[0,0];
Xdes = zeros(3,N);

for i=1:N-1
xvec = X(:,i);
x = xvec(1);
y = xvec(2);
theta = xvec(3);
% theta= wrapTo2Pi(theta);

T0 = T(stage);

l=2.84988;
t=tsteps(i);
basis = [1, t-T0, (t-T0)^2, (t-T0)^3, (t-T0)^4, (t-T0)^5];
dbasis = [0, 1, 2*(t-T0), 3*(t-T0)^2, 4*(t-T0)^3, 5*(t-T0)^4];
ddbasis = [0, 0, 2, 6*(t-T0), 12*(t-T0)^2, 20*(t-T0)^3];

if t>=T(stage+1)
    stage=stage+1;
%     if (stage == 23)
%         abcde = 1;
%     end
    msg = ['start ploting ',num2str(stage),' out of ', num2str(recur), ' sequence of control'];
    disp(msg);
end
xdes = dot(aVecSet(:,stage),basis);
dxdes = dot(aVecSet(:,stage),dbasis);
ddxdes = dot(aVecSet(:,stage),ddbasis);

ydes = dot(bVecSet(:,stage),basis);
dydes = dot(bVecSet(:,stage),dbasis);
ddydes = dot(bVecSet(:,stage),ddbasis);


thetades = atan2(dydes, dxdes);
Xdes(:,i)= [xdes; ydes; thetades];

% The desired state.
xdes_vec = [xdes; ydes; thetades];

% compute the feedforward in the input.
vf = dxdes*cos(thetades) + dydes*sin(thetades);
dthetades = 1/vf*(ddydes*cos(thetades) - ddxdes*sin(thetades));
% dthetades = (ddxdes*dydes-dxdes*ddydes)/(vf^2);
desdelta = atan2(l*dthetades,vf);

A = [ 
      0, 0, -vf*sin(thetades);
      0, 0, vf*cos(thetades);
      0, 0, 0;
    ];
 
B = [ 
      cos(thetades), 0;
      sin(thetades), 0;
      tan(desdelta)/l, vf/l*(tan(desdelta)^2+1);
    ];

Q= [1;1;1].*eye(3);
R = [1;1].*eye(2);

K= lqr(A,B,Q,R);

% Kr = transpose(1./(inv(A-B*K)*B));
% u = -K*xvec + Kr*xdes_vec;

Xe = xvec - xdes_vec;
Xe(3) = wrapToPi(Xe(3));

u = -K*(Xe) + [vf; desdelta];
% set up the limit:
u(1) = limitRange(u(1),4 ,0);
u(2) = limitRange(u(2),0.527,-0.527);

U(:,i+1) = [u(1);
            u(2)];

dxvec = [u(1)*cos(theta);
         u(1)*sin(theta);
         u(1)*tan(u(2))/l];
 
% without noise
X(:,i+1) = dxvec*dt+ X(:,i);

% with noise
% X(:,i+1)= dxvec*dt+ X(:,i) + 0.01*wgn(1,1,0); %+0.01*randn(1);
end

% figure
plot(X(1,:), X(2,:),'LineWidth', 2);
hold on 
% plot(Xdes(1,:), Xdes(2,:), 'LineWidth', 4);


%% printing the velocity versus time.
figure;
for i = 1: recur
    syms t;
    t0 = T(i);
    tF = T(i+1);
    t = t0:0.01:tF;
    a = aVecSet(:,i);
    b = bVecSet(:,i);
    % plot the v-t graph
    trajdX = a(2) + 2*a(3)*(t-t0) + 3*a(4)*((t-t0).^2) + 4*a(5)*((t-t0).^3) + 5*a(6)*((t-t0).^4);
    trajdY = b(2) + 2*b(3)*(t-t0) + 3*b(4)*((t-t0).^2) + 4*b(5)*((t-t0).^3) + 5*b(6)*((t-t0).^4);
    vel = sqrt(trajdX.^2 + trajdY.^2);
    plot(t, vel, 'LineWidth', 3);
    hold on;
    msg = ['plotting ',num2str(i),' out of ', num2str(recur), ' velocity sequences'];
    disp(msg);
end
hold off;   
    
figure;
plot(tsteps,U(1,:));
title('v-t')

figure;
plot(tsteps,U(2,:));
title('delta-t')