function y = round_shift_signed(x, s)
% 与 RTL 一致的“有符号四舍五入右移”
% RTL 对正负数采用对称处理：
%   x >= 0 :  (x + 2^(s-1)) >>> s
%   x <  0 : -((abs(x) + 2^(s-1)) >>> s)

    x = int64(x);
    s = double(s);

    if s <= 0
        y = x;
        return;
    end

    round_const = int64(2^(s - 1));
    divisor     = int64(2^s);

    if x >= 0
        y = idivide(x + round_const, divisor, 'fix');
    else
        y = -idivide(abs(x) + round_const, divisor, 'fix');
    end
end
