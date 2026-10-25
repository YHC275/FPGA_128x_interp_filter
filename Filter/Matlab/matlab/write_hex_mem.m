function write_hex_mem(file_name, data, width)

    data = int64(data(:));
    hex_digits = ceil(width / 4);
    modulus = int64(2^width);

    fid = fopen(file_name, 'w');
    if fid < 0
        error('write_hex_mem: 无法打开文件：%s', file_name);
    end
    cleaner = onCleanup(@() fclose(fid)); 

    for k = 1:numel(data)
        v = data(k);
        if v < 0
            v = v + modulus;
        end
        fprintf(fid, ['%0', num2str(hex_digits), 'X\n'], v);
    end
end
