% System:
k=100;
c=0.1;
m=1;
A=[0 1; -k/m -c/m];
B=[0;1/m];
T=0.01;
time=10;
value=0.2;
Q=[];

RMSE=0;
NMSE=0;
AEloss=0;
L_huber=0;
MSLE=0;
Q_loss=0;
losslist=zeros(6,6);

%Get path (position, time)
[t, q] = getdemo(time,0);
Q=[Q ; q];
[t, q] = getdemo(time,0.3);
Q=[Q ; q];
[t, q] = getdemo(time,0.6);
Q=[Q ; q];
[t, q] = getdemo(time,0.9);
Q=[Q ; q];
[t, q] = getdemo(time,1.2);
Q=[Q ; q];
[t, q] = getdemo(time,1.5);
Q=[Q ; q];

for pathNum=1:6

    q=Q(pathNum,:)';
    %Calculate the position, velocity, and actions using the system:
    q_dot=zeros(length(q),1);
    q_ddot=zeros(length(q),1);
    for n = 1:1:(length(q)-1)
        q_dot(n)=(q(n+1)-q(n))/T;
    end

    for n = 1:1:(length(q)-1)
        q_ddot(n)=(q_dot(n+1)-q_dot(n))/T;
    end

    u=zeros(length(q),1);
    for n = 1:1:(length(q)-1)
        u(n)=(m*(q_dot(n+1)-q_dot(n))/T)+k*q(n)+c*q_dot(n);
    end

    figure (pathNum);
    plot(t, Q(pathNum,:), 'DisplayName', 'Demonstration');

    for y0 =0:0.3:1.5
        % Set up data
        dt = 0.01;  % time step
        N = time/dt;  % number of time steps
        t = dt * (0:N-1);  % time vector

        g = q(length(q));  % goal state

        % Parameters for the demonstration trajectory
        yd = q';  % demonstration trajectory
        yddot = q_dot';
        ydddot = q_ddot';

        % Run the DMP learning and plot the results
        [yd y] = run_dmp(y0, g, yd, yddot, ydddot, 20, [], 0.5, 1, dt, N);


        %Data-Driven errors
        %MSRE over predicted states
        RMSE = RMSE+sqrt(((y-Q(pathNum,:)))*(y-Q(pathNum,:))')/length(Q(pathNum,:));

        %NMSE over predicted action
        % NMSE_u=((u-u_l)'*(u-u_l))/(u'*u);

        %NMSE over predicted over predicted states
        NMSE=NMSE+((y-Q(pathNum,:))*(y-Q(pathNum,:))')/(Q(pathNum,:)*Q(pathNum,:)');

        %Absolute Error Loss over regenirated data
        AEloss=AEloss+sum(abs(Q(pathNum,:)-y))/length(y);

        %Huber Loss
        segma=0.1;
        L=abs(Q(pathNum,:)-y);
        for n=1:1:length(L)
            if (L(n) <=  segma)
                L(n)=L(n)^2;
            else
                L(n)=2*segma*(L(n)-segma/2);
            end

        end
        L_huber=L_huber+sum(L)/length(L);

        %Mean Squared Logarithmic Error (MSLE)
        MSLE=MSLE+((log(Q(pathNum,:)+ones(1,length(Q(pathNum,:))))-log(y+ones(1,length(y))))*(log(Q(pathNum,:)+ones(1,length(Q(pathNum,:))))-log(y+ones(1,length(y))))')/length(y);
        % Quantile Loss
        gamma=2;
        l=abs(Q(pathNum,:)-y);
        Q_loss =Q_loss+(sum(gamma*l(1:round(length(l)/5)))+sum(0.2*gamma*l(round(length(l)/5)+1:round(length(l)*(4/5)))+sum(gamma*l(round(length(l)*(4/5))+1:length(l)))))/length(l);

    end
    losslist(1,pathNum)=RMSE;
    losslist(2,pathNum)=NMSE;
    losslist(3,pathNum)=AEloss;
    losslist(4,pathNum)=L_huber;
    losslist(5,pathNum)=MSLE;
    losslist(6,pathNum)=Q_loss;
    RMSE=0;
    NMSE=0;
    AEloss=0;
    L_huber=0;
    MSLE=0;
    Q_loss=0;
    
end
[v1 bestPathSelectedByRMSE]=min(losslist(1,:));
[v2 bestPathSelectedByNMSE]=min(losslist(2,:));
[v3 bestPathSelectedByAEloss]=min(losslist(3,:));
[v4 bestPathSelectedByL_huber]=min(losslist(4,:));
[v5 bestPathSelectedByMSLE]=min(losslist(5,:));
[v6 bestPathSelectedByQ_loss]=min(losslist(6,:));

list={'Loss Metrics', 'Loss','Location';
       'RMSE',v1 ,bestPathSelectedByRMSE;
       'NMSE',v2 ,bestPathSelectedByNMSE ;
       'AEloss',v3 ,bestPathSelectedByAEloss ;
       'L_huber',v4 ,bestPathSelectedByL_huber ;
       'MSLE',v5 ,bestPathSelectedByMSLE ;
       'Q_loss',v6 ,bestPathSelectedByQ_loss}

function [yd y] = run_dmp(y0, g, yd, yddot, ydddot, alphaz, betaz, alphax, tau, dt, N)
    if isempty(betaz)
        betaz = alphaz / 4;
    end

    % Time vector
    t = dt * (0:N-1);

    % Basis functions
    c = 0:0.1:1;
    sigma = 0.05 * ones(size(c));

    % Learn the parameters
    w = learn(t, yd, yddot, ydddot, alphaz, betaz, alphax, tau, g, y0, dt, N, c, sigma);

    % Set up the forcing term function
    f = @(x) predict(x, w, c, sigma);

    % Reproduce the demonstration via the DMP
    [y, ~, ~] = dmp(y0, alphaz, betaz, g, tau, alphax, f, dt, N);

    % Plot the results

    hold on;
    plot(t, y, '--', 'DisplayName', 'DMP Reproduction');
    hold on
    xlabel('$t$', 'Interpreter', 'latex', 'FontSize', 14);
    ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 14);
    legend;
    title('DMP Learning from Demonstration', 'FontSize', 16);
    grid on;
end

function [y, z, x] = dmp(y0, alphaz, betaz, g, tau, alphax, f, dt, N)
    z = zeros(1, N);
    y = zeros(1, N);
    x = zeros(1, N);
    y(1) = y0;
    x(1) = 1;
    for n = 1:N-1
        x(n+1) = x(n) + dt * (-alphax / tau) * x(n);
        w = f(x(n));
        z(n+1) = z(n) + dt * ((alphaz / tau) * (betaz * (g - y(n)) - z(n)) + w * x(n) * (g - y0));
        y(n+1) = y(n) + dt * (z(n) / tau);
    end
end

function phi = phi_b(x, sigma, c)
    phi = exp(-((x - c).^2) / (2 * sigma^2));
end

function w = learn(t, yd, yddot, ydddot, alphaz, betaz, alphax, tau, g, y0, dt, N, c, sigma)
    x = zeros(1, N);
    x(1) = 1;
    for n = 1:N-1
        x(n+1) = x(n) + dt * (-alphax / tau) * x(n);
    end
    s = x * (g - y0);
    ft = (tau^2) * ydddot - alphaz * (betaz * (g - yd) - tau * yddot);
    w = zeros(1, length(c));
    for i = 1:length(c)
        phi = arrayfun(@(n) phi_b(x(n), sigma(i), c(i)), 1:N);
        Gamma = diag(phi);
        w(i) = (s * Gamma * ft') / (s * Gamma * s');
    end
end

function f = predict(x, w, c, sigma)
    phi = arrayfun(@(ci, si) phi_b(x, si, ci), c, sigma);
    f = (phi * w') / sum(phi);
end


function [t, x] = getdemo(duration,value)
    % Initialize data storage
    t = [];
    x = [];
    
    % Create figure and slider
    f = figure('Name', 'Linear Motion Tracker', 'Position', [100, 100, 600, 400]);
    slider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 1.5, 'Value', value, ...
                       'Position', [150, 50, 300, 20]);
    ax = axes('Parent', f, 'Position', [0.1, 0.3, 0.85, 0.6]);
    hold(ax, 'on');
    ylabel(ax, 'Position');
    xlabel(ax, 'Time (s)');
    title(ax, 'Position vs. Time');
    grid(ax, 'on');

    % Setup timer
    sampleRate = 0.01; % Sample rate in seconds
    numSamples = floor(duration / sampleRate);
    timerObj = timer('ExecutionMode', 'fixedSpacing', 'Period', sampleRate, ...
                     'TasksToExecute', numSamples, 'TimerFcn', @timerCallback, ...
                     'StopFcn', @stopTimer);

    % Timer callback function
    function timerCallback(~, ~)
        % Get the current time and slider value
        currentTime = toc;
        currentPos = get(slider, 'Value');
        t = [t, currentTime];
        x = [x, currentPos];
        plot(ax, t, x, 'b-o');
    end

    % Stop function
    function stopTimer(~, ~)
        delete(timerObj);
        close(f);
    end

    % Start timing and the timer
    tic;
    start(timerObj);
    uiwait(f);
end

