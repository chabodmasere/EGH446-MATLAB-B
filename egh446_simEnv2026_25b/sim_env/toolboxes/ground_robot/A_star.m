function [path, distance] = A_star(start_wp, end_wp, logical_map, scaled)

    if scaled
        map = squeeze(any(reshape(logical_map, 10, 41, 520), 1));
        map = squeeze(any(reshape(map, 41, 10, 52), 2));
        res = 1;
    else
        map = logical_map;
        res = 10;
    end

    [nRows, nCols] = size(map);
    nCells = nRows*nCols;

    gMap    = inf(nRows, nCols);
    visited = false(nRows, nCols);
    parent  = zeros(nRows, nCols);

    [startRow, startCol] = world_to_map(start_wp(1), start_wp(2), res, nRows);
    [endRow,   endCol]   = world_to_map(end_wp(1),   end_wp(2),   res, nRows);

    if map(startRow, startCol) || map(endRow, endCol)
        path = [];
        distance = Inf;
        return
    end

    startId = sub2ind([nRows nCols], startRow, startCol);
    endId   = sub2ind([nRows nCols], endRow, endCol);

    % binary min-heap over f cost, lazy deletion
    cap    = 4*nCells;
    heapId = zeros(cap,1);
    heapF  = inf(cap,1);
    heapN  = 0;

    gMap(startRow, startCol) = 0;
    push(startId, h_cost(startRow, startCol, endRow, endCol));

    offsets = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
    stepLen = [sqrt(2); 1; sqrt(2); 1; 1; sqrt(2); 1; sqrt(2)];

    found = false;

    while heapN > 0
        id = pop();
        [row, col] = ind2sub([nRows nCols], id);

        if visited(row, col)
            continue
        end
        visited(row, col) = true;

        if id == endId
            found = true;
            break
        end

        for k = 1:8
            nRow = row + offsets(k,1);
            nCol = col + offsets(k,2);

            if nRow < 1 || nRow > nRows || nCol < 1 || nCol > nCols
                continue
            end
            if map(nRow, nCol) || visited(nRow, nCol)
                continue
            end

            tentative_g = gMap(row, col) + stepLen(k);

            if tentative_g < gMap(nRow, nCol)
                gMap(nRow, nCol) = tentative_g;
                parent(nRow, nCol) = id;
                f = tentative_g + h_cost(nRow, nCol, endRow, endCol);
                push(sub2ind([nRows nCols], nRow, nCol), f);
            end
        end
    end

    if ~found
        path = [];
        distance = Inf;
        return
    end

    % walk parents back
    ids = endId;
    id  = endId;
    while id ~= startId
        [r, c] = ind2sub([nRows nCols], id);
        id = parent(r, c);
        ids(end+1) = id; %#ok<AGROW>
    end
    ids = flip(ids);

    [rr, cc] = ind2sub([nRows nCols], ids(:));
    path = [(cc - 1)/res, (nRows - rr)/res];

    distance = gMap(endRow, endCol)/res;

    %% heap helpers
    function push(id_, f_)
        heapN = heapN + 1;
        if heapN > cap
            heapId(2*cap) = 0;
            heapF(2*cap)  = inf;
            cap = 2*cap;
        end
        heapId(heapN) = id_;
        heapF(heapN)  = f_;
        i = heapN;
        while i > 1
            p = floor(i/2);
            if heapF(p) <= heapF(i)
                break
            end
            [heapF(p), heapF(i)]   = deal(heapF(i), heapF(p));
            [heapId(p), heapId(i)] = deal(heapId(i), heapId(p));
            i = p;
        end
    end

    function id_ = pop()
        id_ = heapId(1);
        heapId(1) = heapId(heapN);
        heapF(1)  = heapF(heapN);
        heapN = heapN - 1;
        i = 1;
        while true
            l = 2*i;
            r = l + 1;
            s = i;
            if l <= heapN && heapF(l) < heapF(s), s = l; end
            if r <= heapN && heapF(r) < heapF(s), s = r; end
            if s == i
                break
            end
            [heapF(s), heapF(i)]   = deal(heapF(i), heapF(s));
            [heapId(s), heapId(i)] = deal(heapId(i), heapId(s));
            i = s;
        end
    end
end

function d = h_cost(r1, c1, r2, c2)
    d = sqrt((r2-r1)^2 + (c2-c1)^2);
end

function [row, col] = world_to_map(x, y, res, nRows)
    col = round(x*res) + 1;
    row = nRows - round(y*res);
end