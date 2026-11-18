function probability_density = evaluate_normal_probability_density(position_grid, distribution_mean, distribution_variance)
%EVALUATE_NORMAL_PROBABILITY_DENSITY 手写一维正态概率密度。
% 不依赖 Statistics and Machine Learning Toolbox。

distribution_variance = ensure_positive_variance_helper(distribution_variance, 1e-12);
normalization = sqrt(2 * pi * distribution_variance);
normalized_distance_squared = (position_grid - distribution_mean).^2 / distribution_variance;
probability_density = exp(-0.5 * normalized_distance_squared) / normalization;
end
