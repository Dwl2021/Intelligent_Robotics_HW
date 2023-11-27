clc;
clear;
addpath('./utils/')
% 初始位姿
Tseinitial = [0, 0, 1, 0.5518;
              0, 1, 0, 0.0;
             -1, 0, 0, 0.4012;
              0, 0, 0, 1];
          
% 初始物块位姿
Tscinitial = [cos(0), -sin(0), 0, 1.0;
               sin(0), cos(0), 0, 0.0;
               0, 0, 1, 0.0250;
               0, 0, 0, 1];

% 最终物块位姿
Tscfinal = [cos(-pi/2), -sin(-pi/2), 0, 0.0;
             sin(-pi/2), cos(-pi/2), 0, -1.0;
             0, 0, 1, 0.0250;
             0, 0, 0, 1];

% 抓取末端位姿
Tcegrasp = [cos(3*pi/4), 0, sin(3*pi/4), 0.0050;
             0, 1, 0.0, 0.0;
            -sin(3*pi/4), 0, cos(3*pi/4), -0.005;
             0, 0, 0, 1];

% 抓取后的末端位姿（稍微上抬）
Tcestandoff = Tcegrasp;
Tcestandoff(3, 4) = 0.05;

% 生成参考轨迹
k = 400;
result_traj = TrajectoryGenerator(Tseinitial, Tscinitial, Tscfinal, Tcegrasp, Tcestandoff, k);

% 控制输入最大速度
max_speed = 5;

% 参考轨迹长度
N = length(result_traj);

% 初始化位姿
Xd = [result_traj(1,1) result_traj(1,2) result_traj(1,3) result_traj(1,10);
        result_traj(1,4) result_traj(1,5) result_traj(1,6) result_traj(1,11);
        result_traj(1,7) result_traj(1,8) result_traj(1,9) result_traj(1,12);
        0 0 0 1];

% 初始化下一个位姿
Xdnext = [result_traj(2,1) result_traj(2,2) result_traj(2,3) result_traj(2,10);
        result_traj(2,4) result_traj(2,5) result_traj(2,6) result_traj(2,11);
        result_traj(2,7) result_traj(2,8) result_traj(2,9) result_traj(2,12);
        0 0 0 1];

% 初始化当前位形
current_conf = [0 0 0 0 0 0 0 0 0 0 0 0];

% 机械臂末端到基座坐标系的变换矩阵
Tsb_0 = [cos(0) -sin(0) 0 0;
         sin(0) cos(0) 0 0;
         0 0 1 0.0963;
         0 0 0 1];

% 基座坐标系到机械臂基座坐标系的变换矩阵
Tb0 = [1 0 0 0.1662;
        0 1 0 0;
        0 0 1 0.0026;
        0 0 0 1];

% 机械臂末端到末端执行器的变换矩阵
M0e = [1 0 0 0.033;
        0 1 0 0;
        0 0 1 0.6546;
        0 0 0 1];

% 初始末端位姿
Tse = Tsb_0 * Tb0 * M0e;
X = Tse;

% 控制增益
Ki = 5 * eye(6);
Kp = 3 * eye(6);

% 时间步长
dt = 0.01;

% 轮式参数
Blist = [[0; 0; 1; 0; 0.033; 0],...
         [0; -1; 0; -0.5076; 0; 0],...
         [0; -1; 0; -0.3526; 0; 0],...
         [0; -1; 0; -0.2176; 0; 0],...
         [0; 0; 1; 0; 0; 0]];

% 关节角度
thetalist = zeros(5, 1);

% 存储结果
result_conf = zeros(N-1, 13);
result_Xerr = zeros(N-1, 6);

% 控制循环
for i = 1:N-2
    thetalist(:, 1) = current_conf(4:8);
    
    % 计算控制输入
    [control, Xerr] = FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, thetalist);
    
    % 重新排列控制输入
    u = zeros(1, length(control));
    u(1:5) = control(5:9);
    u(6:9) = control(1:4);
    
    % 计算下一时刻位形
    [next_conf] = NextState(current_conf, u, dt, max_speed);
    
    % 存储结果
    result_conf(i, 1:12) = next_conf;
    result_conf(i, 13) = result_traj(i, 13);
    result_Xerr(i, :) = Xerr;
    
    % 更新机械臂末端位姿
    phi = next_conf(1);
    x = next_conf(2);
    y = next_conf(3);
    thetalist(:, 1) = next_conf(4:8);
    
    T0e = FKinBody(M0e, Blist, thetalist);
    Tsb_phi = [cos(phi) -sin(phi) 0 x;
                sin(phi) cos(phi) 0 y;
                0 0 1 0.0963;
                0 0 0 1];
    Tse = Tsb_phi * Tb0 * T0e;
    X = Tse;
    
    % 更新参考位姿
    Xd = [result_traj(i+1, 1) result_traj(i+1, 2) result_traj(i+1, 3) result_traj(i+1, 10);
          result_traj(i+1, 4) result_traj(i+1, 5) result_traj(i+1, 6) result_traj(i+1, 11);
          result_traj(i+1, 7) result_traj(i+1, 8) result_traj(i+1, 9) result_traj(i+1, 12);
          0 0 0 1];
    
    Xdnext = [result_traj(i+2, 1) result_traj(i+2, 2) result_traj(i+2, 3) result_traj(i+2, 10);
              result_traj(i+2, 4) result_traj(i+2, 5) result_traj(i+2, 6) result_traj(i+2, 11);
              result_traj(i+2, 7) result_traj(i+2, 8) result_traj(i+2, 9) result_traj(i+2, 12);
              0 0 0 1];

    % 将下一时刻位形作为当前位形，结束 i 时刻循环，进入 i+1 时刻循环
    current_conf = next_conf;
end

% 绘制误差图
plot(result_Xerr, 'LineWidth', 1)

% 存储结果到 CSV 文件
csvwrite('./result_conf.csv', result_conf);

