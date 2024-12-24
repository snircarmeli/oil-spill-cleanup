clc; clear; close all;

% params

x_i = [2, 3]';
x_f = [-14, 5]';
d = norm(x_f-x_i);
L = 2   ;
n = 10;

if d > n * L
    disp("Error");
end

z = (x_f - x_i) / L;

deg2rad = pi/180;

% res = 1e-5;
% delta = -180:res:180 * deg2rad;
% tol = 1e-7;
% delta_found = 0;
% for i = 1:length(delta)
%     A = calc_A(n, delta(i));
%     v = A \ z;
%     if abs(v(1) ^ 2 + v(2) ^ 2 - 1) < tol
%         delta_found = delta(i);
%     end
% end

delta_1 = 10 * deg2rad;
% delta_2 = 20 * deg2rad;
theta_b = atan2( (x_f(2) - x_i(2)) , (x_f(1) - x_i(1)) );
% 
sigma_c_1 = 0; sigma_s_1 = 0; sigma_c_2 = 0; sigma_s_2 = 0;

for i = 1:n
    sigma_c_1 = sigma_c_1 + cos((i-1)*delta_1);
    sigma_s_1 = sigma_s_1 + sin((i-1)*delta_1);
%     sigma_c_2 = sigma_c_2 + cos((i-1)*delta_2);
%     sigma_s_2 = sigma_s_2 + sin((i-1)*delta_2);
end


z = (x_f-x_i) / L;
% % L = norm(z);
% % z = z / L;
% 
if d > n * L
    disp("Error");
end
% 
M_1 = [sigma_c_1 -sigma_s_1
       sigma_s_1 sigma_c_1];
% M_2 = [sigma_c_2 -sigma_s_2
%        sigma_s_2 sigma_c_2];
% 
v_1 = M_1 \ z;
% v_2 = M_2 \ z;
% 
% % Shooting method
% alpha = sigma_c_1 ^ 2 + sigma_s_1 ^ 2;
% beta = sigma_c_2 ^ 2 + sigma_s_2 ^ 2;
% 
% a = ( z(1) ^ 2 + z(2) ^ 2 ) * alpha;
% b = ( z(1) ^ 2 + z(2) ^ 2 ) * beta;
% c = z(1) ^ 2 * (sigma_c_1 * sigma_c_2 + sigma_s_1 * sigma_s_2) + z(2) ^ 2 * (sigma_s_1 * sigma_s_2 + sigma_c_1 * sigma_c_2);
% 
% coef = [a/(alpha^2) + b/(beta^2) - 2*c/(alpha*beta), 2*c/(alpha*beta) - 2*b/(beta^2), b/(beta^2) - 1];
% 
% lambda = roots(coef);
% 
% v_theta_1 = lambda(1) * v_1 + (1-lambda(1)) * v_2;
% v_theta_2 = lambda(2) * v_1 + (1-lambda(2)) * v_2;
% 
points_1 = zeros(n + 1,2);
points_1(1, :) = x_i;
points_1(end, :) = x_f;
% 
% points_2 = zeros(n + 1, 2);
% points_2(1, :) = x_i;
% points_2(end, :) = x_f;
% 
% theta_i_1 = atan2(v_theta_1(2), v_theta_1(1));
% theta_i_2 = atan2(v_theta_2(2), v_theta_2(1));
% 
% delta_1 = -2 * (theta_i_1 - theta_b) / (n - 1);
% delta_2 = -2 * (theta_i_2 - theta_b) / (n - 1);
% 
% 
% theta_i_1 = theta_b - (n-1) * delta_found/2;
% for i = 2:n+1
%     theta = theta_i_1 + (i-2) * delta_found;
%     points_1(i,:) = points_1(i-1, :) + L*[cos(theta), sin(theta)];
% 
% %     theta = theta_i_2 + (i-2) * delta_2;
% %     points_2(i,:) = points_2(i-1, :) + L*[cos(theta), sin(theta)];
% end
% % 
% % 
% figure('Visible','on');
% plot(x_i(1), x_i(2), 'x'); hold on
% plot(x_f(1), x_f(2), 'x'); hold on
% 
% 
% for i = 1:n
%     x = [points_1(i,1), points_1(i+1,1)];
%     y = [points_1(i,2), points_1(i+1,2)];
%     plot(x, y, '-o', 'LineWidth', 2, 'MarkerSize', 8);
% % 
% %     x = [points_2(i,1), points_2(i+1,1)];
% %     y = [points_2(i,2), points_2(i+1,2)];
% %     plot(x, y, '-o', 'LineWidth', 2, 'MarkerSize', 8);
% 
% end
% axis('equal');





function A = calc_A(n, delta)
    sigma_c = 0; sigma_s = 0;
    for i = 1:n
        sigma_c = sigma_c + cos((i-1) * delta);
        sigma_s = sigma_s + sin((i-1) * delta);
    end
    A = [sigma_c -sigma_s
        sigma_s sigma_c];
end



