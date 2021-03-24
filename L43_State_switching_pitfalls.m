%% A demonstration of the effects of state switching on the systems stability, as shown in lecture 32
% We will plot two stable systems, and show how combining them in one way
% makes the system converge faster and another way makes it unstable
clc
clear all
close all

%Define our two matrixes. They share their eigenvalues
A1 = [-0.03 1; -2 -0.03]
A2 = [-0.03 2; -1 -0.03]

x = [1;1]; % Initial state of the system
tf = 300 % Units of time to run for
% Variable to store the states from every measurement in each case
X1 = sing(A1,x,tf,1); 
X2 = sing(A2,x,tf,2);
X3 = doub(A1,A2,x,tf,3);
X4 = doub(A2,A1,x,tf,4);

function X_ = sing(A,x,tf,j) % Function to get states in cases with no switching
t = 0; dt = 0.01; % We will measure it every tf = 0.01 units
X = zeros(2,tf*100);
X(:,1) = x;
i = 1; % Index variable
while (t<tf)
    x = x + dt.*A*x;
    t = t + dt;
    i = i+1;
    X(:,i) = x;
end
subplot (2,2,j)
p = eig(A);
plot(X(2,:),X(1,:))
ylim([-3 3]);
xlim([-3 3]);
title(j);
txt =['Eigenvalues are ' + string(p(1));' and ' + string(p(2))];
text(-3,2.4,txt);
X_ = 0;
end

function X_ = doub(F,G,x,tf,j)% Function to get states in cases with no switching
t = 0; dt = 0.01; % We want to see how the system behaves for t = 17 units of time, and we will measure it every tf = 0.01 units
X= zeros(2,tf*100);% We will measure it every tf = 0.01 units
X(:,1) = x;
i = 1; % Index variable
A = F;
while (t<tf)
    y = x + dt.*A*x;
    if x(1)*y(1)<0
        A = F;
    elseif x(2)*y(2)<0
        A = G;
    end
    t = t + dt;
    i = i+1;
    X(:,i) = y; x=y;
end
subplot (2,2,j)
plot(X(2,:),X(1,:))
ylim([-3 3]);
xlim([-3 3]);
title(j);
X_=0;
end