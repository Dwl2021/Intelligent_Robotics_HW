function result_traj = TrajectoryGenerator(Tseinitial, Tscinitial, Tscfinal, Tcegrasp, Tcestandoff, k)
    % 生成参考轨迹
    % 参数：
    % Tseinitial 爪子的初始位形
    % Tscinitial 物块的初始位形
    % Tscfinal 物块的最终位形
    % Tcegrasp 抓取时的末端位形
    % Tcestandoff 抓取后的末端位形
    % k 每秒参考轨迹位形数目
    % 返回值：
    % result_traj 生成的参考轨迹

    flag = 0;
    Tf = 100;
    N = k;
    method = 5;

    % 生成初始到standoff的轨迹
    traj = ScrewTrajectory(Tseinitial, Tscinitial * Tcestandoff, Tf, N, method);
    result_traj = [];
    for i = 1:k
        traj_ = cell2mat(traj(i));
        result = traj_to_vector(traj_, flag);
        result_traj = [result_traj; result];
    end

    % 生成standoff到grasp的轨迹
    traj = ScrewTrajectory(Tscinitial * Tcestandoff, Tscinitial * Tcegrasp, Tf, N, method);
    for i = 1:k
        traj_ = cell2mat(traj(i));
        result = traj_to_vector(traj_, flag);
        result_traj = [result_traj; result];
    end

    % 标记flag为1，表示抓取
    flag = 1;
    traj_ = cell2mat(traj(k));
    result = traj_to_vector(traj_, flag);
    for j = 1:k
        result_traj = [result_traj; result];
    end

    % 生成standoff到final_standoff的轨迹
    traj = ScrewTrajectory(Tscinitial * Tcegrasp, Tscfinal * Tcestandoff, Tf, N, method);
    for i = 1:k
        traj_ = cell2mat(traj(i));
        result = traj_to_vector(traj_, flag);
        result_traj = [result_traj; result];
    end

    % 生成final_standoff到final_grasp的轨迹
    traj = ScrewTrajectory(Tscfinal * Tcestandoff, Tscfinal * Tcegrasp, Tf, N, method);
    for i = 1:k
        traj_ = cell2mat(traj(i));
        result = traj_to_vector(traj_, flag);
        result_traj = [result_traj; result];
    end

    % 标记flag为0，表示抓取完成
    flag = 0;
    traj_ = cell2mat(traj(k));
    result = traj_to_vector(traj_, flag);
    for j = 1:k
        result_traj = [result_traj; result(1:12), flag];
    end

    % 生成final_grasp到final_standoff的轨迹
    traj = ScrewTrajectory(Tscfinal * Tcegrasp, Tscfinal * Tcestandoff, Tf, N, method);
    for i = 1:k
        traj_ = cell2mat(traj(i));
        result = traj_to_vector(traj_, flag);
        result_traj = [result_traj; result];
    end
end

function result = traj_to_vector(traj_, flag)
    % 将位姿矩阵转换为向量形式
    R = traj_(1:3, 1:3)';
    p = traj_(1:3, 4)';
    result = [R(:)', p(:)', flag];
end
