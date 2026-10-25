function y = saturate_signed(x, width)
% 把整数 x 饱和到给定位宽的二进制补码范围

    x = int64(x);
    max_v = int64(2^(width - 1) - 1);
    min_v = -int64(2^(width - 1));

    if x > max_v
        y = max_v;
    elseif x < min_v
        y = min_v;
    else
        y = x;
    end
end
