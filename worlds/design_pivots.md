## Design Changes for Experimental Design

### Overview

The original design involved a patrolling Spotter Bot, randomly spawning trash, and sequential trash allocation. These features caused bottlenecks. To correct validity issues in task completion accounting, and improve fairness across comparative setups, we modified the architecture shown below.

The intent is to preserve the original benchmarking goal (Blind vs Swarm vs Coordinated; FIFO vs cost-aware allocation), while improving internal validity and interpretability of results.

---

## Change 1: Remove Spotter as a Detection Bottleneck

### Issue

The Spotter takes an excessively long time to detect rubbish, becoming the dominant bottleneck and limiting Collector utilisation. This prevents the experiment from isolating *coordination and allocation* effects, since mission time is dominated by sensing/coverage latency rather than task allocation efficiency.

### Implementation Change

Assume a **perfect Spotter** to simplify the hypothesis and prevent detection from confounding the results. The physical "Spotter" robot logic has been deprecated and moved to the archive.

* **Supervisor as Spotter:** The `Auctioneer` controller now inherits from the Webots `Supervisor` class instead of a standard `Robot`.
* **Ground Truth Access:** At initialization, the Auctioneer immediately spawns all trash items (either fixed or random) and stores their ground-truth coordinates in `self.trash_locations`.
* **Direct Auctioning:** The system no longer waits for visual detection. The `Auctioneer` actively iterates through the `trash_locations` list and broadcasts `auction_start` messages to collectors immediately.

### Experimental Impact

* **Idle Time Interpretation:** Collector idle time now strictly reflects auction negotiation and travel time, rather than waiting for a Spotter to patrol.
* **Focus:** Coordination performance becomes primarily a function of the allocation algorithm (`select_next_task`) and navigation efficiency.

---

## Change 2: Ensure Trash Pickups Are Counted When Collectors Drive Over Unassigned Trash

### Issue

Collectors sometimes drive over unassigned rubbish while travelling to an assigned target, but these items are not registered as collected. This introduces measurement error and can incorrectly inflate completion time.

### Implementation Change

1.  **Proximity-Based Collection:** The system aims to validate collection based on physical proximity.
    * *Legacy Implementation:* The archived `supervisor.py` calculated the distance between the robot and `active_rubbish` nodes, removing them only if `min_dist < DELETE_THRESHOLD_DIST`.
    * *Current Implementation:* The `collector_coverage.py` logic includes a check where if `dist < 0.5`, the robot stops and sends a collected message.
2.  **Task Accounting:**
    * The `Auctioneer` maintains a `completed_task_ids` set. When a task is collected (even if incidental), it is added to this set and removed from `assigned_task_ids` to prevent re-auctioning.

### Experimental Impact

* **Validity:** Completion time reflects true cleaning performance by acknowledging all successful pickups.
* **Fairness:** Prevents coordinated runs from being unfairly penalised for efficient but "accidental" pickups during transit.

---

## Change 3: Add Nearest-Task Allocation While Keeping Sequential FIFO as Baseline

### Issue

Sequential allocation can be inefficient, creating "zigzag" paths that increase travel distance. With the Spotter bottleneck removed, allocation efficiency becomes the primary differentiator.

### Implementation Change

We implemented a toggleable strategy in `CONFIG.py` (`auction_strategy`) to switch between modes.

* **Sequential (Baseline):** The `select_next_task` function in `auctioneer.py` selects the lowest available `task_id` (FIFO).
* **Nearest-Task Strategy:**
    * The Auctioneer tracks collector positions via `handle_idle` messages.
    * It calculates the Euclidean distance ($d^2 = dx^2 + dz^2$) between every available trash item and every idle collector.
    * It selects the task that minimizes the travel cost for the fleet.

### Experimental Impact

* **Direct Comparison:** Allows for a clean within-architecture comparison where only the allocation heuristic changes ("sequential" vs "nearest_task").
* **Hypothesis Testing:** Directly tests if cost-aware assignment reduces total distance travelled and mission time.

---

## Change 4: Control Trash Placement to Ensure Fair Comparisons Across Setups

### Issue

Randomly spawned trash locations can make setups non-comparable. One configuration may be evaluated on an “easy” distribution while another faces a clustered distribution.

### Implementation Change

A `trash_spawn_strategy` variable was added to `CONFIG.py` to control this behavior.

* **Random Mode:** The `Auctioneer` samples random free cells from the occupancy grid to spawn 10 items (`_spawn_random_rubbish`).
* **Fixed Mode:** The `Auctioneer` loads coordinates from a hardcoded `FIXED_TRASH_LOCATIONS` list (e.g., `(0.5, 0.5)`, `(1.0, 1.0)`) to ensure identical start conditions for every trial.

### Experimental Impact

* **Variance Reduction:** Enables fair, direct comparisons between Baseline/Swarm/Coordinated setups by keeping the environment constant.
* **Reproducibility:** Fixed locations ensure that performance differences are due to the architecture, not the random seed.

---

## Change 5: Automated Data Logging & Visualization
### Issue
On-screen metrics were transient and difficult to capture accurately for post-run analysis.

### Implementation Change
Decouple data collection from real-time display using a persistent logging module. The `Logger` class (in `dual_logger.py`) captures console output and saves it to markdown files.

Separated log outputs for he Auctioneer (bids/assignments) and Collector (navigation/ASCII maps) are saved to their respective controller directories in an embeddede `logs\` folder for granular analysis.

### Experimental Impact
This feature preserves precise timing and distance data that was previously lost after the simulation ended. This eliminates the potential for human error in manual result recording.

## Updated Hypotheses

* **H1:** Increasing robot count without coordination reduces completion time but shows diminishing returns due to interference.
* **H2:** Coordinated allocation reduces completion time relative to uncoordinated swarming under identical task sets.
* **H3:** Cost-aware allocation (**Nearest-Task**) significantly reduces total distance travelled relative to **Sequential (FIFO)** allocation within the coordinated architecture.

---

## Control Variables to Report Explicitly

* **Spawn Strategy:** Controlled via `CONFIG.py` (`"fixed"` or `"random"`).
* **Allocation Strategy:** Controlled via `CONFIG.py` (`"sequential"`, `"nearest_task"`).
* **Controller Stack:** All setups use the same A* `GlobalPlanner` and `SimpleAvoidance` local planner.
* **Task Count:** Fixed at 10 items per run.