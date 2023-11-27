function [control, Xerr] = FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, thetalist)
    % 控制器：计算控制输入和误差
    
    % 计算目标位姿变化的期望末端速度
    Vd = se3ToVec(1/dt * MatrixLog6(Xd\Xdnext));
    
    % 计算当前位姿与目标位姿的误差
    Xerr = se3ToVec(MatrixLog6(X\Xd));
    
    % 计算末端速度
    AdXinvXdVd = Adjoint(X\Xd) * Vd;
    V = AdXinvXdVd + Kp * Xerr + Ki * (Xerr * dt);
    
    % 机械臂关节信息
    Blist = [[0; 0; 1; 0; 0.033; 0],...
             [0; -1; 0; -0.5076; 0; 0],...
             [0; -1; 0; -0.3526; 0; 0],...
             [0; -1; 0; -0.2176; 0; 0],...
             [0; 0; 1; 0; 0; 0]];
    
    % 计算机械臂雅可比矩阵
    Jarm = JacobianBody(Blist, thetalist);
    
    % 末端和基座坐标系的变换矩阵
    Tb0 = [1 0 0 0.1662;
           0 1 0 0;
           0 0 1 0.0026;
           0 0 0 1];
    T0b = inv(Tb0);
    
    % 末端到基座的逆变换
    M0e = [1 0 0 0.033;
           0 1 0 0;
           0 0 1 0.6546;
           0 0 0 1];
    T0e = FKinBody(M0e, Blist, thetalist);
    Te0 = inv(T0e);
    
    % 轮式参数
    r = 0.0475; % 轮子半径
    l = 0.47/2; % L的小写l，与下面的1作区分
    w = 0.3/2;  % 车宽的一半
    
    % 轮式雅可比矩阵
    F6 = [0 0 0 0;
          0 0 0 0;
          -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);
          1 1 1 1;
          -1 1 -1 1;
          0 0 0 0;] * r/4;
    Jbase = Adjoint(Te0*T0b) * F6;
    
    % 完整雅可比矩阵
    Je = [Jbase Jarm];
    
    % 计算控制输入
    control = pinv(Je) * V;
end
