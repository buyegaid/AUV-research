function y_next = first_order_lpf(y_prev, x, fc, dt)
% 一阶低通滤波: fc cutoff in Hz
if fc <= 0
    y_next = x;
    return;
end
tau = 1/(2*pi*fc);
alpha = dt/(tau + dt);
y_next = (1-alpha)*y_prev + alpha*x;
end
