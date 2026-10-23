"""CPU graph/forest model. Timing is neither GPU nor production CPU evidence."""
from collections import deque


def graph(colors, active):
    """Exact tile partitions via serial union-find, independent of GPU ping-pong."""
    height, width = colors.shape
    forest = [i+1 if active.flat[i] else 0 for i in range(colors.size)]
    edges, boundary = [], []

    def root(i):
        while forest[i]-1 != i:
            i = forest[i]-1
        return i

    for y in range(height):
        for x in range(width):
            if not active[y, x]:
                continue
            a = y*width+x
            for dx, dy in ((1, 0), (-1, 1), (0, 1), (1, 1)):
                nx, ny = x+dx, y+dy
                if not (0 <= nx < width and ny < height and active[ny, nx]):
                    continue
                if colors[y, x] != colors[ny, nx]:
                    continue
                b = ny*width+nx
                edges.append((a, b))
                if (x//16, y//16) != (nx//16, ny//16):
                    boundary.append((a, b))
                else:
                    ra, rb = root(a), root(b)
                    forest[max(ra, rb)] = min(ra, rb)+1
    forest = [root(i)+1 if p else 0 for i, p in enumerate(forest)]
    return forest, edges, boundary


def reference(colors, active):
    """Independent 8-neighbor flood-fill; canonical root is smallest pixel+1."""
    height, width = colors.shape
    roots = [0]*colors.size
    for seed in range(colors.size):
        if roots[seed] or not active.flat[seed]:
            continue
        roots[seed] = seed+1
        queue = deque([seed])
        while queue:
            pixel = queue.popleft()
            y, x = divmod(pixel, width)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = x+dx, y+dy
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    other = ny*width+nx
                    if roots[other] or not active[ny, nx] or colors[ny, nx] != colors[y, x]:
                        continue
                    roots[other] = seed+1
                    queue.append(other)
    return roots


def solve(initial, original_edges, boundary, *, copyback, rounds=32):
    """Immutable A walks -> optional B copy -> serial equivalent of atomic min.

    Walk fault semantics reproduce root(); sticky faults cannot be erased by a
    subsequent zero copy. Serial min on frozen B equals any atomic-min order.
    """
    forest = list(initial)
    count = len(forest)
    fault = reads = copied = 0

    def snapshot():
        nonlocal fault, reads
        roots = []
        for parent in forest:
            r = parent
            reads += 1
            for _ in range(count):
                if not r or r > count:
                    break
                following = forest[r-1]
                reads += 1
                if following == r:
                    break
                if following == 0 or following >= r:
                    fault |= 4
                    r = 0
                    break
                r = following
            else:
                fault |= 4
                r = 0
            if r > count:
                fault |= 4
                r = 0
            roots.append(r)
        return roots

    for _ in range(rounds):
        roots = snapshot()
        if copyback:
            forest[:] = roots
            copied += 4*count
        for a, b in boundary:
            left, right = roots[a], roots[b]
            if left and right and left != right:
                high, low = max(left, right), min(left, right)
                forest[high-1] = min(forest[high-1], low)
    roots = snapshot()  # production final compress; no extra copy
    if any(roots[a] and roots[b] and roots[a] != roots[b] for a, b in original_edges):
        fault |= 1
    return {'roots': roots, 'fault': fault, 'parent_reads': reads, 'copy_bytes': copied}
