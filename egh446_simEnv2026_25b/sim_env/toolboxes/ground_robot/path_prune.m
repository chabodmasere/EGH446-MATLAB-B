function pruned = path_prune(path)
if size(path,1) < 3
    pruned = path;
    return
end

keep = true(size(path,1),1);
for i = 2:size(path,1)-1
    a = path(i,:) - path(i-1,:);
    b = path(i+1,:) - path(i,:);
    if abs(a(1)*b(2) - a(2)*b(1)) < 1e-9
        keep(i) = false;
    end
end
pruned = path(keep,:);
end