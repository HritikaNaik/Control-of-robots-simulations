%% A demonstration of pole placement, as shown in lecture 32
% We see that eigen values with positive real parts go to infinity,
% those with no real part just osscillate
% and only those with a negetive real part converge
% Furthermore, the complex part determines rate of convergence
clc
clear all
close all
% Define a system given by

%% x' = A*x + B*u (State)
%% u = -K*x (Feedback)
% K is a gain matrix with K = [k1 k2] here
A = [2,0;1,-1];
B = [1;1];
% Lets pick p, our desired eigenvalues
% Complex Eigen values must be conjugate pairs
p = [-0.3+5i,-0.3-5i]
% Some other eigen values to try out:
%p = [-0.5+i,-0.5-i];[+i,-i];[-1, -1];[-0.5 + 0.866i,-0.5-0.866i];[-0.1127, -0.8873];[-0.5,-1];[-0.5,1];[-0.03-i,-0.03+i]
K = place(A,B,p) % Inbuilt function that performs pole (eigenvalue) placement for given A, B and p

x = [1;1]; % Initial state of system
t = 0; tf = 17; dt = 0.01; % We want to see how the system behaves for t = 17 units of time, and we will measure it every tf = 0.01 units
X = zeros(2,1700); % Variable to store the states found from every measurement
X(:,1) = x;
i = 1; % Index variable
while (t<tf)
    x = x + dt.*(A-B*K)*x;
    t = t + dt;
    i = i+1;
    X(:,i) = x;
end

%% Plotting
T = 0:dt:17.01;
tiledlayout(1,2);
grid on

%% Change in x1 with time
nexttile % We put X1 and X2 on the same plot
plot(T,X(1,:))
hold on
xlim([0 17]);

%% Change in x2 with time
plot(T,X(2,:))
hold off;
% X1 and X2 always have very similiar behavior

%% Change in x1 and x2 with respect to reach other
nexttile
plot(X(2,:),X(1,:))
hold off;