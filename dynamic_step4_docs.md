# Dynamic Step 4 — Spatio-Temporal A* Path Generation

---

## Prerequisites — What You Need to Know First

### 1. Static A* Recap
In the static pipeline, A* searched on a 2D grid:
- **State:** `(row, col)` — where the intruder is
- **Blocked:** `fov_mask[row, col]` — fixed forever
- **Path:** sequence of `(row, col)` cells

### 2. Why Static A* Fails for Dynamic Cameras
A rotating camera means a cell that is blocked **right now** might be free in 2 seconds — and vice versa. Static A* has no concept of time — it either always blocks a cell or never does. It cannot handle "block this cell only between t=10 and t=25".

### 3. The 4D Problem
To handle time-varying visibility, the search space gains a third dimension — time:
```
Static : state = (row, col)          → 2D search
Dynamic: state = (row, col, time)    → 3D search space = "4D problem"
                                        (x, y, z already 3D in world space,
                                         adding time makes it 4D)
```

### 4. The FoV Stack
Instead of one `fov_mask.npy`, Dynamic Step 3 produced T masks — one per frame:
```
fov_stack[t, row, col] = True   →  camera sees this cell at time t
fov_stack[t, row, col] = False  →  camera does NOT see this cell at time t
```
Shape: `(T, H, W)` — T frames, H rows, W columns.

### 5. The "Look at Next Frame" Principle
When the intruder moves from cell A to cell B, they arrive at B at the **next timestep** `t+1`. So the safety check is:
```
"Will I be seen when I ARRIVE at B?" → check fov_stack[t+1, B]
NOT
"Am I seen now at A?" → fov_stack[t, A]
```

### 6. Waiting
Since time always advances, the intruder can **wait in place** — stay at `(row, col)` while time ticks forward. This lets them lurk behind cover until the camera rotates away, then move. This is a new action that doesn't exist in static A*.

### 7. Time Budget
The intruder must reach the goal **before frame T**. If the rotation cycle has 145 frames, the intruder has at most 145 time steps to get from start to goal. Paths that are geometrically possible but take too long are rejected.

---

## Inputs
- `terrain_mask.npy` — static, from Step 3
- `obstacle_mask.npy` — static, from Step 3
- `mask_frames/fov_mask_0000.npy` ... `fov_mask_N.npy` — per-frame FoV, from Step 3
- Interactive band drawing — start and goal polylines drawn by user

---

## Outputs
```
paths_spacetime.npy          — paths as (r, c, t) tuples
paths_spacetime.json         — same in JSON
stats.json                   — run metadata and parameters
paths_colored.png            — all paths projected to 2D BEV (t dropped)
all_masks_and_paths.png      — FoV union over all frames + paths overlaid
coverage_stats.txt           — terrain/blind area coverage over full rotation
frames_all/frame_0000.png    — per-frame: FoV at t + path prefixes up to t
        .../frame_0001.png
        ...
paths_timeslice.mp4          — video assembled from per-frame PNGs
```

---

## Params
```
ALLOW_DIAG            = True    — 8-connected movement (4 cardinal + 4 diagonal)
ALLOW_WAIT            = True    — intruder can stay in place (time still advances)
DIAG_COST             = 1.414   — cost of diagonal move
N_POINTS              = 10      — clicks per polyline for band drawing
TARGET_STARTS         = 100     — sample ~100 starts from drawn band
TARGET_GOALS          = 100     — sample ~100 goals from drawn band
GOALS_PER_START_LIMIT = 500     — max goals tried per start
MAX_PATHS             = 100     — hard cap on total paths found
RENDER_FRAMES         = 10      — how many frame PNGs to render
MAKE_VIDEO            = True    — assemble frames into MP4
VIDEO_FPS             = 24
```

---

## Part 1 — Load and Stack FoV Masks

```python
fov_list = []
for fp in sorted(glob.glob(fov_mask_*.npy)):
    fov_list.append(np.load(fp).astype(bool))

fov_stack = np.stack([pad(f, H, W) for f in fov_list], axis=0)
# shape: (T, H, W)
T = fov_stack.shape[0]
```

All T per-frame masks loaded and stacked into a single 3D array. Padding ensures all frames have identical shape.

`fov_stack[t, r, c] = True` means camera sees cell `(r, c)` at frame `t`.

Also pads terrain and obstacle to same `(H, W)` shape.

---

## Part 2 — Walkable Static Grid

```python
walkable_static = (terrain | obstacle)
```

Same rule as static — obstacles are passable for the intruder. Only camera FoV blocks movement, but now FoV is time-varying so it's checked per-step inside A*, not here.

---

## Part 3 — Core A* Function: `astar_spacetime`

This is the heart of the file. Everything else is setup or post-processing.

### 3a. State Definition
```python
state = (row, col, t)
```
Three integers. Each unique `(r, c, t)` combo is a distinct node in the search graph. Two visits to the same cell at different times are completely separate states.

### 3b. Heuristic
```python
def heuristic(a, b):
    return max(abs(a[0]-b[0]), abs(a[1]-b[1]))   # Chebyshev distance
```
Chebyshev distance = max of row-diff and col-diff. Used instead of Manhattan because with diagonal moves allowed, the minimum steps to reach `(goal_r, goal_c)` from `(r, c)` is `max(|Δr|, |Δc|)` not `|Δr| + |Δc|`.

Time dimension is **ignored** in the heuristic — it only measures 2D spatial distance. This is still admissible (never overestimates) because the intruder needs at least this many steps to reach the goal regardless of time.

### 3c. Earliest Safe Entry Time
```python
def earliest_safe_t(r, c):
    safe_idx = np.flatnonzero(~fov_stack[:, r, c])
    return int(safe_idx[0]) if safe_idx.size else -1
```
Scans all T frames for a start cell — finds the **first frame where this cell is not covered** by any camera. The intruder cannot enter the scene while the camera is watching their start cell.

Returns `-1` if the cell is covered in every single frame → impossible starting point, skip it.

### 3d. Main Loop
```python
def astar_spacetime(start_rc, goal_rc, t0):
    start = (start_rc[0], start_rc[1], int(t0))
    g_cost = {start: 0.0}
    came   = {}
    pq = [(heuristic(start_rc, goal_rc), start)]

    while pq:
        _, cur = heapq.heappop(pq)
        r, c, t = cur

        # Goal check — only spatial, any time is fine
        if r == goal_rc[0] and c == goal_rc[1]:
            return reconstruct_path(cur, came)

        # Time budget exhausted
        if t + 1 >= T:
            continue

        # Expand neighbours
        for nr, nc in neighbors(r, c):
            if not (0 <= nr < H and 0 <= nc < W): continue
            if not walkable_static[nr, nc]: continue

            # THE KEY CHECK — look at next frame
            if fov_stack[t+1, nr, nc]:
                continue   # will be seen when arriving → blocked

            nxt = (nr, nc, t+1)
            new_g = g_cost[cur] + 1.0
            if nxt not in g_cost or new_g < g_cost[nxt]:
                g_cost[nxt] = new_g
                came[nxt] = cur
                f = new_g + heuristic((nr, nc), goal_rc)
                heapq.heappush(pq, (f, nxt))

    return None   # no path found within time budget
```

### 3e. The Move Set
```python
MOVES = [(-1,0), (1,0), (0,-1), (0,1)]   # 4 cardinal
      + [(-1,-1), (-1,1), (1,-1), (1,1)]  # 4 diagonal (if ALLOW_DIAG)
      + [(0, 0)]                           # wait in place (if ALLOW_WAIT)
```

**Wait move `(0, 0)`** — `nr = r`, `nc = c`. The intruder stays in the same cell but time advances to `t+1`. The check `fov_stack[t+1, r, c]` determines if waiting here is safe next frame. If the camera is about to rotate to cover this cell, waiting is blocked. If the camera is rotating away, waiting is allowed.

### 3f. Goal Check
```python
if r == goal_rc[0] and c == goal_rc[1]:
    return path
```
Only checks spatial coordinates — any arrival time is acceptable. The intruder wins as long as they reach the goal row and column, regardless of which frame they arrive at.

### 3g. Time Budget Check
```python
if t + 1 >= T:
    continue
```
If we're at the last frame, no more moves possible. The rotation cycle has ended — stop expanding this state.

---

## Part 4 — Impossibility Gate

Before running A* for each start-goal pair:
```python
dist_lb = heuristic(start, goal)
if t0 + dist_lb >= T:
    continue   # geometrically impossible to reach in time
```

Chebyshev distance is the minimum number of steps to reach the goal. If `t0 + min_steps >= T`, the intruder can't possibly reach the goal before the time budget runs out — skip this pair entirely without running A*. Massive speedup when many pairs are impossible.

---

## Part 5 — Band Drawing and Path Generation Loop

### 5a. Draw bands
Same interactive polyline drawing as static:
- Click 10 points → rasterize with `skimage.draw.line` → filter walkable → subsample to ~100 cells

### 5b. Per-start setup
```python
t0 = earliest_safe_t(int(s[0]), int(s[1]))
if t0 < 0:
    continue   # this start is always covered → skip
```
Each start cell gets its own `t0` — the first frame it's safe to enter. This means different start cells begin their journey at different times.

### 5c. Per-pair A*
```python
for s in start_cells:
    t0 = earliest_safe_t(s)
    for g in goal_cells:
        if t0 + heuristic(s, g) >= T: continue   # impossibility gate
        p = astar_spacetime(s, g, t0)
        if p: paths.append(p)
```

Standard nested loop like static Step 4 — one A* per start-goal pair, up to `GOALS_PER_START_LIMIT` goals per start and `MAX_PATHS` total.

---

## Part 6 — Outputs

### 6a. paths_spacetime.npy / .json
Each path is a list of `(r, c, t)` tuples — the intruder's position at each timestep. Example:
```
[(45, 120, 3), (45, 121, 4), (45, 122, 5), (46, 123, 6), ...]
```
Row and col move as the intruder walks. Time always increases by 1 per step.

### 6b. paths_colored.png
Projects paths to 2D by dropping `t`:
```python
for (r, c, t) in path:
    path_mask[r, c] = True
```
Shows where intruders travel spatially. Multiple timesteps at the same cell merge into one pixel.

### 6c. all_masks_and_paths.png
```python
fov_union = np.any(fov_stack, axis=0)
```
`fov_union[r, c] = True` if ANY frame covered this cell. Shows the full sweep of cameras across the entire rotation cycle, overlaid with paths.

### 6d. Per-frame PNGs + MP4
For each frame `t`:
- Show FoV at exactly `t` (white)
- Show path prefixes — all path cells where `tt <= t` (black)

So at t=0 you see empty terrain. As t increases you see the intruders moving through the scene while the camera rotates. Assembled into an MP4 for easy review.

### 6e. coverage_stats.txt
```python
fov_union = np.any(fov_stack, axis=0)
covered_area = np.sum(terrain & fov_union)
blind_area   = terrain_area - covered_area
```
Answers: across the entire rotation cycle, what fraction of terrain was ever covered by a camera? The remainder is the persistent blind spot — terrain that no camera ever sees regardless of rotation angle.

---

## Static vs Dynamic A* — Full Comparison

| Aspect | Static A* | Dynamic A* |
|---|---|---|
| State | `(r, c)` | `(r, c, t)` |
| Search space | 2D grid | 3D grid (x, y, time) |
| FoV input | one `fov_mask.npy` | `fov_stack[T, H, W]` |
| Blocked check | `fov_mask[r, c]` | `fov_stack[t+1, nr, nc]` |
| Which frame checked | N/A — fixed | **next** frame `t+1` |
| Can wait | ❌ | ✅ `(0,0)` move |
| Start time | always t=0 | `earliest_safe_t(start)` |
| Time budget | none | must reach goal before T |
| Heuristic | Manhattan | Chebyshev (ignores time) |
| Path output | `[(r,c), ...]` | `[(r,c,t), ...]` |
| Impossibility gate | ❌ | ✅ `t0 + dist >= T` |
| Visualization | static BEV | per-frame PNGs + MP4 |

---

## Why Check `t+1` and Not `t`?

This is the most important design decision in the file.

When the intruder is at `(r, c, t)` and decides to move to `(nr, nc)`:
- They **leave** `(r, c)` at time `t`
- They **arrive** at `(nr, nc)` at time `t+1`

The camera's position at time `t` is irrelevant — the intruder is already leaving that cell. What matters is whether the camera is watching `(nr, nc)` at time `t+1` — when the intruder actually lands there.

If we checked `fov_stack[t, nr, nc]` instead:
- We'd be asking "is `(nr, nc)` safe right now?" — but the intruder won't be there until next frame
- The camera could rotate to cover `(nr, nc)` between `t` and `t+1`
- The intruder would walk right into the camera's view

Checking `t+1` ensures the intruder is safe when they actually arrive — not just when they decided to move.
