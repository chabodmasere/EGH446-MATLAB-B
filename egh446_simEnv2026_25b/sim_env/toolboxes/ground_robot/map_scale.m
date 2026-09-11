function smallMap = map_scale(logical_map, scale)
    smallMap = logical_map(1:scale:end, 1:scale:end);
end