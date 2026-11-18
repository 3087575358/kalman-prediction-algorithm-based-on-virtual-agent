function limited_value = clamp_value_to_limits_helper(value, lower_limit, upper_limit)
%CLAMP_VALUE_TO_LIMITS_HELPER 将数值限制在给定闭区间内。

if lower_limit > upper_limit
    error('clamp_value_to_limits_helper:InvalidLimits', ...
        'lower_limit 不能大于 upper_limit。');
end

limited_value = max(lower_limit, min(upper_limit, value));
end
