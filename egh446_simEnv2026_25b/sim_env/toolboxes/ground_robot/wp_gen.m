function wp_list = wp_gen(x_max, y_max, N, x0, logical_map)

    wp_list = zeros(N, 2);
    n = 0;

    while n < N
        c = [randi(x_max*10),randi(y_max*10)];
        if logical_map(c(2),c(1)) == 1
            continue
        end
        c = c/10;
        if any(all([wp_list(1:n,:); x0] == c, 2))
            continue
        end
        
        n = n + 1;
        wp_list(n,:) = c;
    end
end