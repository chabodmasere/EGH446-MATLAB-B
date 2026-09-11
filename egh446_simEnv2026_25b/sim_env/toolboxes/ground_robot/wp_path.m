function [path, distance] = wp_path(wp_route, occ)
%WP_PATH  Stitch an ordered waypoint route into one continuous path.
%
%   OCC is the fine-resolution map, already inflated by the required
%   clearance. Returns the full A* path with no simplification.

    path     = wp_route(1,:);
    distance = 0;

    for i = 1:size(wp_route,1)-1
        [leg, d] = A_star(wp_route(i,:), wp_route(i+1,:), occ, false);

        if isempty(leg)
            error('wp_path:noRoute', 'No path between waypoint %d and %d.', i, i+1);
        end

        path     = [path; leg(2:end,:)];
        distance = distance + d;
    end
end