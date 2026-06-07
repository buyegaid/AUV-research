function out = fal(e, alpha, delta)
% Vectorized fal with safe handling
% e: n x 1
% alpha: scalar in (0,1)
% delta: small positive scalar
ae = abs(e);
out = zeros(size(e));
small = (ae <= delta);
% linear inside deadzone (scaled)
out(small) = e(small) ./ (delta.^(1 - alpha));
% power outside
out(~small) = sign(e(~small)) .* (ae(~small).^alpha);
end
