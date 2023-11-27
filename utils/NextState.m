function next_conf = NextState(current_conf, control, dt, max_speed)
    % 更新智能车状态
    
    % 限制控制速度不超过最大速度
    for i = 1:9
        if max_speed == 0
            break
        end
        if abs(control(i)) > max_speed
            control(i) = max_speed * sign(control(i));
        end
    end

    % 解包当前状态
    phi = current_conf(1);  % 车体朝向角度
    J = current_conf(4:8);  % 车体速度
    theta = current_conf(9:12);  % 关节角度

    % 解包控制量
    J_dot = control(1:5);  % 车体速度变化率
    theta_dot = control(6:9);  % 关节角速度

    % 计算下一个状态的速度和角速度
    dJ = J_dot * dt;
    dtheta = theta_dot * dt;

    % 计算车体位移和旋转
    l = 0.47 / 2;  % 轴距的一半
    w = 0.3 / 2;   % 车宽的一半
    r = 0.0475;    % 轮子半径
    F = (r / 4) * [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
                   1, 1, 1, 1;
                   -1, 1, -1, 1];
    Vb = F * dtheta';

    % 计算智能车的新位置
    if Vb(1) == 0
        dqb = [0; Vb(2); Vb(3)];
    else
        dqb = [Vb(1); 
               (Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1);
               (Vb(3)*sin(Vb(1))-Vb(2)*(cos(Vb(1))-1))/Vb(1)];
    end
    dq = [1, 0, 0;
          0, cos(phi), -sin(phi);
          0, sin(phi), cos(phi)] * dqb;

    % 更新状态
    next_conf(1:3) = current_conf(1:3) + dq';
    next_conf(4:8) = J + dJ;
    next_conf(9:12) = theta + dtheta;
end
