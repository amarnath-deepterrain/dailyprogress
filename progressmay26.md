# progress may 26

## step 4 path generation
- Required: terrain obstacle fov masks as npy files
- output: paths.npy has all paths from start bands to end bands
- start band  and end band : selected manually mostly except multi a start band

Multi a star start and end bands:
- take top 200 rows of terrain true cells for start
- take bottom 370 rows of terrain true cells for goal


# Multi-Source A* — Logic Breakdown

## Overview
Finds one shortest blind-spot path per start cell to any goal cell.
All starts run simultaneously in a shared priority heap, each tracked independently.

---

## Inputs
- `terrain_mask.npy` — walkable ground cells
- `obstacle_mask.npy` — blocked cells
- `fov_mask.npy` — camera-visible cells (intruder must avoid)
- `SELECT_MASK.npy` — precomputed band of start + goal candidates

---

## Step 1 — Build Walkable Grid
- Load all three masks, pad to same shape
- `walkable = (terrain | obstacle) & (~fov)`
- A cell is walkable if it is terrain or obstacle **AND** not visible to any camera
- `blocked = ~walkable` — what A* treats as walls

---

## Step 2 — Generate SELECT_MASK (separate script)
- Scan terrain column by column
- For each column, take the topmost 30 terrain rows → mark as start band (cyan)
- For each column, take the bottommost 30 terrain rows → mark as goal band (yellow)
- Filter: start rows must be within top 200 rows, goal rows within bottom 370 rows
- Save combined binary mask as `SELECT_MASK.npy`

---

## Step 3 — Extract Start and Goal Cells
- Load `SELECT_MASK`
- `start_cells` = all True cells in mask where `row < 200` AND `walkable`
- `goal_cells` = all True cells in mask where `row >= H - 370` AND `walkable`
- Subsample both with stride 200 to keep count manageable (~20 each)
- Result: list of `(row, col)` tuples

---

## Step 4 — Seed All Starts into Shared Heap
- For each start cell, assign a unique `idx` (start ID)
- Set `g_scores[(idx, cell)] = 0` — cost to reach this cell from start `idx` is 0
- Set `came_from[(idx, cell)] = None` — no parent yet
- Push `(f=0, g=0, cell, idx)` into the shared priority heap
- All starts enter simultaneously — this is what makes it multi-source

---

## Step 5 — Main A* Loop
Loop condition: `while heap not empty AND not all starts have reached a goal`

On each iteration:
- Pop cell with lowest `f = g + h` from heap
- `f` = total estimated cost, `g` = actual cost so far, `h` = heuristic
- Also pop `sid` — which start this expansion belongs to

---

## Step 6 — Goal Check
- If popped cell is in `goal_set` AND this `sid` hasn't reached a goal yet:
  - Mark `sid` as reached
  - Reconstruct path by following `came_from[(sid, cur)]` backwards from goal to start
  - Reverse and save to `paths`
  - `continue` — don't expand this cell, move to next heap item
  - Other starts keep running unaffected

---

## Step 7 — Neighbour Expansion
For each of 8 neighbours (4 cardinal + 4 diagonal):
- Skip if out of bounds
- Skip if `blocked`
- Compute step cost: `1.0` for cardinal, `1.414` for diagonal
- `cand_g = g + step`
- Key = `(sid, neighbour)` — per-start tracking
- If this is a cheaper path to this neighbour for this start:
  - Update `g_scores[(sid, neighbour)]`
  - Update `came_from[(sid, neighbour)] = current`
  - Compute heuristic: `h = min Manhattan distance to any goal cell`
  - Push `(cand_g + h, cand_g, neighbour, sid)` to heap

---

## Step 8 — Path Reconstruction
When a start `sid` hits a goal:
```
goal → came_from[(sid, goal)] → ... → came_from[(sid, start)] → None
```
Follow the chain backwards, collect all cells, reverse → shortest path for this start

---

## Step 9 — Save and Visualize
- Save all paths to `paths_1.npy` and `paths_1.json`
- Draw paths on BEV image:
  - Path body = black (1px per cell)
  - Start cell = cyan circle
  - Goal cell = yellow circle
- Save `paths_colored_1.png`
- Save `coverage_stats_1.txt`:
  - Total terrain pixels
  - Covered by FoV (camera sees)
  - Blind spot area (intruder can use)

---

## Key Design Choices

| Choice | Reason |
|---|---|
| `(sid, cell)` as key | Keeps each start's path independent — they don't overwrite each other |
| Shared heap | All starts expand together — more efficient than N separate A* runs |
| Stop per-start on goal hit | Each start gets its own optimal path, not just the global best |
| Manhattan distance heuristic | Admissible (never overestimates) → A* remains optimal |
| Stride 200 subsampling | Keeps start/goal count manageable without losing coverage |

---

## Output
- One path per start cell
- Each path = shortest route from that start to its nearest goal through blind spots only
- If a start has no reachable goal → no path saved for it (unreachable)
