function wp_list = wp_gen(x_max, y_max, N, x0, logical_map)

    nRows = size(logical_map, 1);
    wp_list = zeros(N, 2);
    n = 0;

    while n < N
        c = [randi(x_max*10)-1, randi(y_max*10)-1]/10;
        [row, col] = world_to_map(c(1), c(2), 10, nRows);

        if logical_map(row, col)
            continue
        end
        if any(all([wp_list(1:n,:); x0] == c, 2))
            continue
        end

        n = n + 1;
        wp_list(n,:) = c;
    end
end

function [row, col] = world_to_map(x, y, res, nRows)
    col = round(x*res) + 1;
    row = nRows - round(y*res);
end