Goal is to place minimum number of cameras such that all of the paths generated in previous step are eliminated
## Camera_opti.py
### inputs:
  - terrain_mask.npy — walkable ground
  - obstacle_mask.npy — obstacles
  - fov_mask.npy — existing camera FoV from Step 3
  - paths.npy — intrusion paths from Step 4
### outputs:
- camera_N_view.png — BEV image after each camera placement
- camera_N_stats.txt — coverage stats after each camera

### params:
```
CAMERA_RANGE   = 200       — how far rays travel
FOV_ANGLE_DEG  = 30        — cone width
ANGLE_SAMPLES  = 150       — rays per cone (uniform spacing)
AZIMUTH_DEG    = 270       — fixed direction (west)
MAX_CAMERAS    = 1         — only places 1 camera
```
### essence of the logic : 
- place poles in the walkable terrain area (walkable = terrain & (~obstacle) & (~current_vis))

> loop through max cameras.. the below is inside the loop of max cameras
- loop through all walkable cells.
        -- place pole . calculate score
        --  score = gain_area + len(newly_covered_paths) 
        -- gain_area = area of the new cone that is on terrain (not obstacle) and that which is not covered in any previous fov
        -- newly_covered_paths : for path in all_paths : if any pixel of path is in cone : newly_covered_paths.append(path) 
        basically if even a pixel is in the new fov the path is blocked
        --  update the results and move to next walkable cell
        ``` current_vis |= best_cone      # update FoV
covered_paths.update(...)     # mark paths as done```
---
## camera_opti_3cam.py
### same logic as above only difference is you have infinite cameras instead of just max cameras 
- logic is same. just a infinite while loop instead of looping through max MAX_CAMERAS
---
## optimisation. py

### inputs:
- terrain_mask.npy
- obstacle_mask.npy
- fov_mask.npy — existing FoV from Step 3
- paths.npy — intrusion paths from Step 4

### outputs:
- camera_N_view.png — BEV after each placement
- camera_N_stats.txt — per camera stats
- camera_positions.csv — all camera positions
- camera_positions.json — same in JSON

### params:
```
CAMERA_RANGE   = 200     — ray travel distance
FOV_ANGLE_DEG  = 30      — cone width
ANGLE_SAMPLES  = 150     — rays per cone
AZIMUTH_DEG    = 270     — fixed direction west
K_PIXELS       = 20      — min path cells in cone to count as covered
```
### logic (differences/similarity with previous approach)
- biggest difference is score computation . we no longer take the new  gain_area for the cone...
- ``` gain = len(newly_covered)  # score = number of *new* paths covered```
- also since area is not the objective we CAN place a camera in a existing fov. reason is our objective is new paths covered... 
- another change is *covered_paths* :
      --- previously a path is covered IF a single pixel of the path is in the cone...
      --- now a path is covered IF only its has >=K_PIXELS in the cone...

-  the reason for the K-pixel rule isn't just strictness for its own sake. It's to avoid false coverage. If a camera just barely clips the edge of a path (1-2 pixels), the intruder can still effectively use that route — they just avoid that one cell. K=20 means the camera has to meaningfully intersect the path, not just graze it.
---

## optimisation single path .py
> note: this solves a bit of a different problem altogether because we only have a single path instead of multiplw like others
> another note: No Fovs No path masks.. we are mixing step3 and 4 into this directly
### inputs:
- terrain_mask.npy
- obstacle_mask.npy
- Interactive band drawing (start + goal polylines)
### outputs:
Outputs per iteration:

  - iter_N/path.npy + path.json — the path found this iteration
  - iter_N/path_overlay.png — scene with current path shown
  - iter_N/camera.json — camera placed this iteration
  - iter_N/cone.npy — cone mask
  - iter_N/camera_overlay.png — scene after camera placed

### Final outputs:

  - cameras.csv + cameras.json — all camera positions
  - paths.json — all paths found across iterations
  - final_overlay.png — everything combined
  - summary.json — full run metadata
### params:
``` 
FOV_DEG          = 30
AZIMUTH_DEGS     = [0,30,60...330]  — tests ALL 12 orientations
CAMERA_RANGE     = 200
ANGLE_SAMPLES    = 150
CANDIDATE_STRIDE = 1                — exhaustive scan
BBOX_MARGIN      = 20               — candidate area around path
MAX_ITERS        = 200

```
### logic simple version :
- intruder path is first made using multi source A* (starts, goals , blocked)
- based on that path defender places pole... find best camera to cover this path of intruder
- next iteration : intruder path again (blocked is updated from the previous iteration defender pole)
- defender places pole again
- ``` score = int(np.sum(cone & path_mask))``` How many pixels of this specific path fall inside this cone. Not total area, not all paths — just this iteration's path. Higher = better.

> think of it as a game between intruder and defender
- intruder win condition:  defender is stuck ``` if best["score"] <= 0:
    break```
- defender win condition : ``` if path_rc is None:
    break```
- Tie condition: MAX_ITERS number of loops completed. so MAX_ITERS number of cameras/poles placed by the end.

### detailed logic :
> this is done per iteration:
* step1 intruder move:
  * build blocked grid (copy of existing fov mask which is set to 0 initially)
  * run multi source A* and get path
  * check if path exists 

* step 2 defender move:
  * get mask of the path from step 1 
  * candidate positions : instead of taking the whole terrain only take positions which are close to the path
      ```
        rmin = min(path rows) - BBOX_MARGIN   # 20 cells above path
        rmax = max(path rows) + BBOX_MARGIN   # 20 cells below path
        cmin = min(path cols) - BBOX_MARGIN   # 20 cells left of path
        cmax = max(path cols) + BBOX_MARGIN   # 20 cells right of path

      ```
  * for each candidate position and each of the 12 azimuths 
      ```
      for (x, y) in cand_iter:
        for az in AZIMUTH_DEGS:   # [0, 30, 60, ..., 330]
      ```
  * Cast rays
      ``` cone = cast_rays_from(x, y, az, FOV_DEG, CAMERA_RANGE, ANGLE_SAMPLES, obstacle)```
  * absolutely no overlap is allowed with the current fov (vis_union)
      ```
      if np.any(cone & vis_union):
        continue   # skip this candidate entirely
      ```
  * score the camera
  * track best score 
      ```
      if score > best["score"]:
        best = (x, y, az, cone, score) 
      ```
  * if no valid camera found defender lost break out of the main loop
  * Commit the best camera:
      ```
      camera_positions.append((best["x"], best["y"]))
      camera_azimuths.append(best["az"])
      camera_cones.append(best["cone"])
      vis_union |= best["cone"]   # ← this is what changes blocked for next iteration
      ```

---
## optim .py 
### inputs:
- terrain_mask.npy
- obstacle_mask.npy
- Interactive band drawing (cached to bands_cache.json)

### Outputs:
- iter_000/paths.json — initial paths before any camera
- iter_000/snapshot.png — initial scene
- iter_NNN/paths.json — remaining paths after camera N
- iter_NNN/snapshot.png — scene after camera N
- placement_log.csv — iter, x, y, azimuth, gain, paths_before, paths_after
- cameras.json — all placed cameras
- fov_mask.npy — final combined FoV

### Params:
```
CAMERA_RANGE     = 200
FOV_ANGLE_DEG    = 30
ANGLE_SAMPLES    = 150
AZIMUTH_CHOICES  = [270]   — only west direction
K_PIXELS         = 1       — any 1 pixel overlap counts
MAX_CAMERAS      = 50      — hard cap
MIN_GAIN_PATHS   = 1       — must cover at least 1 path
CANDIDATE_STRIDE = 20      — scan every 20th cell for speed
```

### LOGIC main:
#### Pre main loop:
- Load masks (current vis initialised with 0s)
- Load or draw bands (need to do it only ones after which they are saved)
- generate paths iteration 000:
      ```paths = generate_paths(current_vis, start_cells, goal_cells)```

#### Main loop:
- check if paths still exist if not then break (no paths are existing so out job is done)
- convert paths to sets
- Build candidate locations for pole placing (terrain and not obstacle cells) with stride.. 
      ```
                for (x, y) in tqdm(cand_coords, desc=f"iter {it}: candidates"):
                        for az in AZIMUTH_CHOICES:
      ```

- Main step : score locations:
-- cast cone
-- make sure cone has no overlap with current_vis
-- score with k rule  (how many paths have at least k pixels in the cone)
-- track the best :
      ``` if newly > best[0]: best = (newly, x, y, az, cone)``` newly is the score here
- stopping check : 
          ```
            if best is None or best[0] < MIN_GAIN_PATHS:
                print("No candidate improves ≥1 paths. Ending.")
                break
          ```
- commit camera:
      ```
        current_vis |= bcone
        camera_positions.append((bx, by))
      ```
- rerun A* with updated Fov :
      ```
        paths = generate_paths(current_vis, start_cells, goal_cells)
        paths_after = len(paths)
      ```
- save stuff and log to csv
> if paths_after is 0 we are done...

---
## Greedy by Marginal Area with Path-Hit Guard
### inputs:
- terrain_mask.npy
- obstacle_mask.npy
- fov_mask.npy — existing FoV from Step 3 (unlike optimization_singlepath.py which starts empty)
- Interactive band drawing (start + goal polylines)
### outputs:
  - step_XX.png — BEV snapshot after each camera placed
  - placements_log.csv — step, center, azimuth, path length, marginal area, coverage stats
  - coverage_stats.txt — human readable stats appended after each step
  - selected_cameras.json — all placed cameras
  - final_summary.png — final scene
### Params:
```
FOV_DEG      = 30
RANGE_PX     = 200
AZIMUTHS     = [0,30,60...330]   — all 12 orientations
GRID_STRIDE  = 25                — candidate density
K_MAX        = 50                — max cameras
NO_OVERLAP   = True              — strict no cone overlap
MAX_PATH_SKIPS_PER_ROUND = 10000     — how many paths to try before giving up
CAMERA_ON_TERRAIN_ONLY = True    — candidates only on terrain
```
> Important notes before logic:
* ```unblocked_terrain = terrain & (~current_vis)```
* Scoring criterion of a cone(candidate) :
    - first filter no overlap with current fovs
    - must have one pixel intersection with the current path
    - ``` gain = int((cand["mask"] & unblocked_terrain).sum())``` how much area of the whole unblocked terrain is the candidate covering
* Candidate mask computation done upfront and saved:
```
candidates = []
for (r,c) in cand_centers:        # every 25th cell on terrain
    for az in AZIMUTHS:            # all 12 orientations
        m = cone_mask((r,c), az, FOV_DEG, RANGE_PX, H, W)
        candidates.append({
            "center": (r,c),
            "az":     int(az),
            "mask":   m,           # H×W bool array
            "flat":   np.flatnonzero(m.ravel())  # 1D indices for fast intersection
        })

```
> ### Path skipping (Important)
* Why it exists:
In a tug-of-war loop, the defender must place a camera that covers the current shortest path. But two constraints can make a path uncoverable:

  - No-overlap constraint — new cone cannot overlap any existing cone
  - No valid terrain — no walkable camera position exists near the path

* As more cameras are placed and more of the grid is covered by cones, the probability of hitting one of these uncoverable paths increases. Without path skipping, hitting a single uncoverable path terminates the entire game — even if dozens of other coverable paths still exist. This is a premature and incorrect stopping condition.

* What it does:
Instead of stopping when a path is uncoverable, the algorithm temporarily skips that path and tries the next shortest one. It keeps trying alternative paths until either a coverable one is found or the skip budget is exhausted.

### Main logic:

* LOAD masks : terrain, obstacle, fov
* Build initial blocked grid ~((terrain | obstacle) & (~current_vis))
    - Walkable = terrain or obstacle AND not in any camera FoV.
    - Blocked = everything else. Intruder cannot walk on blocked cells.
* Draw start and goal bands interactively
* Precompute ALL candidate cones upfront
* Initialize tracking variables:
    ``` 
      selected    = []                          # placed cameras
      union_flat  = np.array([], dtype=np.int64)  # union of all placed cone flats
    ```
#### Main LOOP:
> while step < K_MAX:
> Each outer iteration = one camera placement attempt.
* Reset path skipping state : Fresh skip state every round — previous round's skipped paths are walkable again.

* Inner loop ``` while tried_paths < MAX_PATH_SKIPS_PER_ROUND:```
      * Find current shortest path ```path = astar_multi(starts, goals, blocked | blocked_skip)  ```
      * if no path exists we are done. goal achieved.
      * if path found : tried paths +=1 and flatten the path to path_flat
      * Start scoring candidates
      * check overlap if yes skip it
      * check if it has at least one pixel intersection with path_flat if no skip it
      * score the candidate using the formula above
      * keep track of the best candidate:
          ```
                    if gain > best_gain:
                      best_gain = gain
                      best_idx  = idx
          ```
      * If no valid candidate is found (if best_idx==-1) path is uncoverable. then make the path blocked by adding it to blocked
      * if valid candidate found commit it increase steps also union_fov
      * Record the data
---
