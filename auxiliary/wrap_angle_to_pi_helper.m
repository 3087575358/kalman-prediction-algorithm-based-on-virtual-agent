function wrapped_angle = wrap_angle_to_pi_helper(angle)
%WRAP_ANGLE_TO_PI_HELPER 将任意角度折叠到 [-pi, pi) 区间。
% 输入和输出单位均为 rad；支持标量、向量和矩阵。

wrapped_angle = mod(angle + pi, 2 * pi) - pi;
end
