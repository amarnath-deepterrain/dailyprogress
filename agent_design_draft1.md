# LLM-Based Surveillance Camera Placement: Agent Design (Draft)

## Goal

Design a surveillance camera placement system as a two-agent game:

* Intruder Agent (LLM)
* Defender Agent (LLM)

The environment consists of:

* Terrain mask
* Obstacle mask
* Existing cameras
* Existing FoV coverage
* Path generation algorithms (A*, MSMG, K-shortest paths, etc.)

The objective is to study strategic interactions between an intruder attempting to traverse the environment and a defender attempting to place cameras that prevent successful intrusion.

---

# Core Design Principle

LLMs should not replace path planning algorithms.

Algorithms such as:

* A*
* MSMG
* K-shortest paths

remain responsible for generating valid paths through the environment.

LLMs are responsible for strategic decision making.

In other words:

* Algorithms generate possibilities.
* LLMs choose among possibilities.

---

# Environment

The environment maintains:

* Terrain
* Obstacles
* Existing cameras
* Current FoV mask
* Path history
* Camera placement history

The environment is responsible for:

1. Generating candidate paths.
2. Validating camera placements.
3. Updating FoV after camera placement.
4. Maintaining game state.

---

# Intruder Agent

## Purpose

The intruder chooses which available path to attempt.

The intruder does NOT generate paths.

Path generation remains the responsibility of A* or other path generation algorithms.

---

## Intruder Observation

The intruder receives:

* Candidate paths
* Path metadata
* Current camera locations
* Current FoV coverage
* Previous successful/blocked paths
* Previous defender actions

Possible path metadata:

* Path length
* Corridor label (west, center, east, etc.)
* Number of cameras nearby
* Percentage of path exposed
* Percentage of path hidden by obstacles
* Start region
* Goal region

Example:

Path 1:

* Length: 120m
* Exposure: High
* Corridor: Center

Path 2:

* Length: 140m
* Exposure: Low
* Corridor: West

Path 3:

* Length: 135m
* Exposure: Medium
* Corridor: East

---

## Intruder Action

The intruder outputs:

```json
{
    "chosen_path_id": 2
}
```

Meaning:

"I choose Path #2 from the available candidate paths."

---

## Intruder Objective

Reach the goal while minimizing:

* Detection risk
* Camera exposure
* Travel cost

The intruder may also adapt based on defender history.

Example reasoning:

* Defender has heavily protected the west corridor.
* Switch to the center corridor.

---

# Defender Agent

## Purpose

Place a new surveillance camera after observing the intruder's chosen path.

The defender attempts to prevent future intrusions while using as few cameras as possible.

---

## Defender Observation

The defender receives:

### Current Environment

* Terrain
* Obstacles
* Existing cameras
* Current FoV coverage

### Intruder Information

* Chosen path
* Path metadata

### History

* Previously chosen intruder paths
* Previously blocked paths
* Previous camera placements
* Previous defender actions

---

## Defender Action

The defender outputs:

```json
{
    "x": 523,
    "y": 188,
    "azimuth": 270
}
```

Meaning:

* Place a camera at coordinate (523, 188)
* Face the camera west (270°)

The environment validates the placement.

Possible validation strategies:

* Reject invalid positions.
* Snap to nearest valid terrain cell.
* Re-prompt the defender.

---

## Defender Objective

Choose camera placements that:

* Block current intrusion attempts.
* Anticipate future intrusion attempts.
* Maximize surveillance coverage.
* Minimize total cameras used.

The defender should think strategically rather than greedily.

Example:

Greedy strategy:

* Place camera directly on current path.

Strategic strategy:

* Place camera at a bottleneck likely to affect multiple future paths.

---

# Game Loop

## Round 0

Environment initialized:

* Terrain loaded
* Obstacles loaded
* No cameras present

---

## Round N

### Step 1: Generate Candidate Paths

Using algorithm X:

* A*
* MSMG
* K-shortest paths
* Other path generation methods

Generate:

```text
P1
P2
P3
...
Pn
```

with metadata.

---

### Step 2: Intruder Turn

Intruder observes:

* Candidate paths
* Metadata
* Current environment

Intruder selects:

```json
{
    "chosen_path_id": k
}
```

---

### Step 3: Defender Turn

Defender observes:

* Chosen path
* Environment state
* History

Defender outputs:

```json
{
    "x": ...,
    "y": ...,
    "azimuth": ...
}
```

---

### Step 4: Environment Update

Environment:

* Validates placement
* Adds camera
* Updates FoV

---

### Step 5: Generate New Candidate Paths

Algorithm X runs again using updated FoV.

New candidate paths are produced.

---

### Step 6: Repeat

Continue until:

* No feasible paths remain, OR
* Camera budget exhausted, OR
* Maximum rounds reached

---

# Why A* Is Still Required

A* is not an agent.

A* is an environment tool.

A* provides:

* Feasible path generation
* Route optimization
* Navigation constraints

The LLM should not generate raw path coordinates.

Instead:

A* generates candidate paths.

The intruder chooses among them.

This preserves geometric correctness while enabling strategic decision making.

---

# Current Working Hypothesis

The most promising formulation is:

Path Generator (A*/MSMG/K-shortest)
↓
Candidate Paths
↓
LLM Intruder chooses one path
↓
LLM Defender places one camera
↓
Environment updates
↓
Repeat

This creates a genuine agent-vs-agent game while preserving the existing path-planning infrastructure.
