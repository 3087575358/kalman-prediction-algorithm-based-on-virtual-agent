function safe_variance = ensure_positive_variance_helper(variance_value, minimum_variance)
%ENSURE_POSITIVE_VARIANCE_HELPER 防止概率计算中出现零方差或负方差。

if nargin < 2
    minimum_variance = 1e-12;
end

safe_variance = max(real(variance_value), minimum_variance);
end
