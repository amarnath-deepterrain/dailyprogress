# Dynamic pole placement 
---
# Step 2
> step 1 is the same, get ground segmented.ply and obstacle segmented.ply
* Static Step 2 :
  * get valid camera positions. get azimuths . Rank them based on the score (coverage area)
  * get a rays. ply for each camera

* Dynamic step 2: get the location from static step 2 first..(the most optimal location) .. azimuth keeps changing. 
  * fix time/frame and compute azimuth

## Inputs
```
ground_segmented.ply — from Step 1
obstacle_segmented.ply — from Step 1
Hardcoded camera positions (from static step 2)
```
## Outputs:
```
frames_all/
    frame_0000/
        camera_raypoints_cam00.ply   ← dense ray points, for Step 3 masking
        camera_cone_cam00.ply        ← LineSet, for visualization only
        camera_raypoints_cam01.ply
        camera_cone_cam01.ply
        camera_raypoints_cam02.ply
        camera_cone_cam02.ply
        frame_0000.png               ← composite 3D render all cameras
    frame_0001/
        ...
    frame_N/
        ...

```
## Params:
```
FOV_DEG            = 30.0       — camera field of view
TILT_DEG           = 5.0        — downward tilt of camera
MAX_RANGE          = 150.0      — max ray travel distance (metres)
NUM_RAYS           = 10000      — rays per cone per frame
POLE_HEIGHT        = 10.0       — camera height above ground
RAY_STEP           = 0.5        — step size along each ray
HIT_THRESHOLD      = 0.30       — KDTree hit distance (metres)
OMEGA_DEG          = 5.0        — rotation speed (degrees/second)
DT                 = 0.5        — time per frame (seconds)
DETERMINISTIC_CONE = True/False — uniform vs random ray sampling
```

 
## Part 1 — Camera Positions
 
Three hardcoded camera base positions from static step 2:
```python
CAMERA_POSITIONS = [
    np.array([-8.10, 105.74, -9.28]),    # Camera 0
    np.array([286.59,  65.07, 39.98]),   # Camera 1
    np.array([center[0], center[1], center[2]])  # Camera 2 at scene center
]
```
 
All three cameras rotate in sync — same azimuth at every frame. Each camera is raised by `POLE_HEIGHT` at render time:
```python
cam_pos = cam_base + [0, 0, POLE_HEIGHT]
```
 
---
 
## Part 2 — Azimuth Schedule (Fixing Time)
 
The outermost loop iterates over frames — each frame corresponds to a fixed point in time. The azimuth at that frame is what determines where the camera is pointing.
 
### Waypoints and Legs
 
The rotation journey is defined as a sequence of azimuth waypoints:
```python
AZI_SEQ    = [270,   0, 270, 180, 270]   # azimuth waypoints
AZI_DELTAS = [+90, -90, -90,  +90]       # direction of each leg
```
 
A **leg** is one continuous rotation segment between two waypoints — the camera rotates in one direction until it reaches the next waypoint, then changes direction.
 
```
Start  → 270°
Leg 1  → rotate +90° → reach   0°
Leg 2  → rotate -90° → reach 270°
Leg 3  → rotate -90° → reach 180°
Leg 4  → rotate +90° → reach 270°  (back to start)
```
 
### Frames Per Leg
 
Each leg takes a certain number of frames depending on rotation speed and timestep:
```python
def frames_for_leg(deg_delta):
    return int(np.ceil(abs(deg_delta) / (OMEGA_DEG * DT)))
    # = ceil(90 / (5.0 * 0.5)) = ceil(36) = 36 frames per leg
```
 
Breaking it down:
- `OMEGA_DEG * DT` = degrees per frame = 5.0 × 0.5 = 2.5°/frame
- `abs(deg_delta) / degrees_per_frame` = frames needed for this leg
- `ceil` to round up — never cut a leg short
```
LEG_FRAMES = [36, 36, 36, 36]   # 4 legs × 36 frames each
T_TOTAL    = sum(LEG_FRAMES) + 1 = 145 frames
```
 
The `+1` accounts for the initial frame at the starting azimuth (270°) before any rotation begins.
 
### Getting Azimuth at Any Frame — Piecewise Linear Interpolation
 
Given frame number `t`, `azimuth_at(t)` returns the exact azimuth at that moment:
 
```python
def azimuth_at(t):
    remaining = t
    cur = AZI_SEQ[0]              # start at 270°
 
    for k, nframes in enumerate(LEG_FRAMES):
        if remaining <= nframes:
            # t falls inside this leg
            step_per_frame = AZI_DELTAS[k] / nframes
            return cur + step_per_frame * remaining
 
        # t is beyond this leg — advance to next
        cur += AZI_DELTAS[k]
        remaining -= nframes
 
    return AZI_SEQ[-1]
```
 
**Example — finding azimuth at t=50:**
```
Leg 1 has 36 frames → t=50 is beyond leg 1
  cur      = 270 + 90 = 0°
  remaining = 50 - 36 = 14
 
Leg 2 has 36 frames → remaining=14 falls inside leg 2
  step_per_frame = -90 / 36 = -2.5°/frame
  azimuth = 0 + (-2.5 × 14) = -35° → 325°
```
 
The logic is simply — figure out which leg `t` falls in, then linearly interpolate within that leg. No trigonometry, just proportional steps.
 
---

## Part 3 — Cone Generation
 
Two modes controlled by `DETERMINISTIC_CONE`:
 
### Random (DETERMINISTIC_CONE = False)
```python
u = np.random.rand(n_rays) - 0.5   # random scatter in [-0.5, 0.5]
v = np.random.rand(n_rays) - 0.5
theta = az   + u * fov
phi   = tilt + v * (fov * 0.5)
```
Random scatter within the cone — same as static Step 2. Can cause flickering between frames since rays land in slightly different cells each frame.
 
### Deterministic (DETERMINISTIC_CONE = True)
```python
g  = int(np.sqrt(n_rays))           # grid size, e.g. 100 for 10000 rays
us = (np.arange(g) + 0.5) / g - 0.5  # evenly spaced in [-0.5, 0.5]
vs = (np.arange(g) + 0.5) / g - 0.5
```
Uniform `g × g` grid sampling — evenly spaced angles across the cone. No randomness. Gives **stable consistent frames** — important for dynamic because the same cells get stamped every frame, no flickering in the FoV mask.
 
Both modes produce `(n_rays, 3)` unit direction vectors.
 
---
 
## Part 4 — Ray Casting
 
```python
def cast_cone(cam_pos, ray_dirs):
```
 
For each ray:
- Step along the ray at `RAY_STEP` intervals from camera position
- At each step, query KDTree for nearest scene point
- If distance < `HIT_THRESHOLD` → hit, stop ray
- Collect every sampled point → `sampled_all` (dense, for Step 3 masking)
- Collect final endpoint → `line_points` (sparse, for visualization LineSet)
Returns:
- `sampled_all` — all points along all rays, used by Step 3 to stamp the FoV mask
- `line_points` + `line_indices` — LineSet for 3D visualization only
---
 
## Part 5 — Main Frame Loop
 
```python
for t in range(T_TOTAL):                          # outer loop fixes time
    azi = azimuth_at(t)                           # azimuth at this frame
 
    for cam_idx, cam_base in enumerate(CAMERA_POSITIONS):   # inner loop = cameras
        cam_pos  = cam_base + [0, 0, POLE_HEIGHT]
        ray_dirs = generate_cone_directions(azi, TILT_DEG, FOV_DEG, NUM_RAYS)
        pts, ls_points, ls_idx = cast_cone(cam_pos, ray_dirs)
 
        # save dense raypoints for Step 3
        o3d.io.write_point_cloud(f"frame_{t}/camera_raypoints_cam{cam_idx}.ply", ...)
        # save LineSet for visualization
        o3d.io.write_line_set(f"frame_{t}/camera_cone_cam{cam_idx}.ply", ...)
 
    save_frame_png(...)     # composite PNG of all 3 cameras at this frame
```
 
**Key point — outer loop fixes time:**
The outer loop iterates over frames. Each frame = one fixed moment in time. Everything inside the frame loop (azimuth, ray casting, saving) happens at that specific timestep. This is what makes it a temporal sequence rather than a single static snapshot.
 
**All cameras share the same azimuth per frame** — they rotate in sync. In a more advanced version each camera could follow an independent rotation schedule.
 
---
 
## Why Deterministic Cone Matters for Dynamic
 
In static Step 2 random sampling was fine — you only generate one FoV mask. In dynamic, you generate T_TOTAL masks and stack them into `visibility[t, row, col]`. If random sampling causes different cells to be stamped each frame, a cell might appear visible at t=5 and invisible at t=6 even if the camera hasn't moved — pure sampling noise. Deterministic sampling eliminates this by always stamping the same cells for the same azimuth angle.
 
---
 
## Difference vs Static Step 2
 
| | Static Step 2 | Dynamic Step 2 |
|---|---|---|
| Camera positions | scored + selected by 2c/2d | hardcoded |
| Azimuth | best per camera from 2c | follows rotation schedule |
| Output | 3 ray PLY files | 3 × T_TOTAL ray PLY files |
| FoV | fixed forever | changes every frame |
| Ray sampling | random | deterministic (for stability) |
| Feeds into | Step 3 single mask | Step 3 builds `visibility[t,r,c]` |
 
---
# Step 3 (Masking)
---
## Inputs:
```
  ground_segmented.ply — from Step 1
  obstacle_segmented.ply — from Step 1
  frames_all/frame_0000/camera_raypoints_*.ply ... frame_N/ — from Step 2
```
---
## Outputs:
```
  OUT_ROOT/
    terrain_mask.npy         — static, one file
    obstacle_mask.npy        — static, one file
    grid_meta.json           — grid coordinate metadata
    mask_frames/
        fov_mask_0000.npy    — FoV at t=0
        fov_mask_0001.npy    — FoV at t=1
        ...
        fov_mask_0144.npy    — FoV at t=144
    previews/
        static_combined.png  — terrain + obstacle BEV
        fov_frame_0000.png   — BEV at t=0
        fov_frame_0001.png   — BEV at t=1
        ...
```
---

## Params:
```
  GRID_RES     = 0.5    — metres per grid cell
  SAVE_PREVIEW = True   — save PNG previews
```
---
## Part 1 — Load Base Point Clouds
```
  ground_pcd = o3d.io.read_point_cloud(GROUND_PLY)
  obst_pcd   = o3d.io.read_point_cloud(OBSTACLE_PLY)
  g_pts = np.asarray(ground_pcd.points)
  o_pts = np.asarray(obst_pcd.points)
```
## Part 2 Define Grid Bounds (Save it to meta data of grid)
```
  all_xy = np.vstack([g_pts[:, :2], o_pts[:, :2]])
  min_x, min_y = all_xy.min(axis=0)
  max_x, max_y = all_xy.max(axis=0)

  W = int(np.ceil((max_x - min_x) / GRID_RES))
  H = int(np.ceil((max_y - min_y) / GRID_RES))

```
## Part 3 Rasterize Helper
```
  def rasterize_xy_to_mask(xy, H, W, min_x, min_y, res, out_mask):
    cols = np.floor((xy[:, 0] - min_x) / res).astype(np.int64)
    rows = np.floor((xy[:, 1] - min_y) / res).astype(np.int64)
    valid = (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
    out_mask[rows[valid], cols[valid]] = 1

```
## Part 4 terrain and obstacle masks
```
  terrain_mask  = np.zeros((H, W), dtype=np.uint8)
  obstacle_mask = np.zeros((H, W), dtype=np.uint8)
  rasterize_xy_to_mask(g_pts[:, :2], H, W, min_x, min_y, GRID_RES, terrain_mask)
  rasterize_xy_to_mask(o_pts[:, :2], H, W, min_x, min_y, GRID_RES, obstacle_mask)
  terrain_mask[obstacle_mask == 1] = 0   # obstacle wins overlap

```
## Part 5 Per-Frame FoV Masks

```
  frame_dirs = sorted(glob.glob(os.path.join(CONES_ROOT, "frame_*")))

  for fdir in frame_dirs:
      ply_files = sorted(glob.glob(os.path.join(fdir, "camera_raypoints_*.ply")))

      fov_mask = np.zeros((H, W), dtype=np.uint8)   # fresh mask per frame

      for pf in ply_files:
          pcd = o3d.io.read_point_cloud(pf)
          pts = np.asarray(pcd.points)
          rasterize_xy_to_mask(pts[:, :2], H, W, min_x, min_y, GRID_RES, fov_mask)

      fov_mask[obstacle_mask == 1] = 0   # FoV can't penetrate obstacles

      frame_idx = int(os.path.basename(fdir).split("_")[-1])
      np.save(f"fov_mask_{frame_idx:04d}.npy", fov_mask)


```
* 5a. Fresh empty FoV mask:
* 5b. Load all camera raypoints for this frame
* 5c. Stamp each camera's rays into the mask
* 5d. Resolve obstacle overlap
* 5e. Save

---

