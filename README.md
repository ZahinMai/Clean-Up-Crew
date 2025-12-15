# Cafeteria Clean-Up-Crew: Benchmarking Multi-Robot Coordination Strategies

**A Webots-based benchmarking framework for evaluating task allocation and coordination strategies in autonomous cleaning robots.**

## Overview

This project is **not just a Multi-Agent System (MAS) simulator** — it is a **controlled benchmarking framework** designed to **systematically compare coordination strategies** for automated cleaning in a cafeteria environment.

Rather than demonstrating a single “smart” solution, the system enables **side-by-side evaluation** of four paradigms under identical conditions:

1. **Baseline:** A single autonomous robot (no coordination).
2. **Swarm:** Multiple robots operating without explicit task allocation.
3. **Auction-Based Coordination:** Three distinct auction strategies.

All experiments are run in the **same world, with the same obstacles, navigation stack, and trash distribution**, allowing performance differences to be attributed directly to **coordination strategy**, not environmental variance.

---

## Benchmarking Scenarios

Each simulation run operates in one of the following controlled setups:

| Mode | Description |
|----|----|
| **BASELINE** | One robot performs all cleaning tasks sequentially |
| **SWARM** | Multiple robots act independently without coordination |
| **AUCTION – Sequential** | Centralised auction, FIFO task ordering |
| **AUCTION – Nearest Task** | Centralised auction optimising fleet-wide travel distance |
| **AUCTION – Random** | Randomised task selection for robustness testing |

The default and primary experimental focus is **auction-based coordination**, with the baseline and swarm modes acting as **control conditions**.

---

## System Architecture

The system follows a **Spotter–Collector-inspired architecture**, evolved into a **benchmark-ready Multi-Agent System** with strict role separation and logging.

### The Team (v2.0 Architecture)

1. **The Auctioneer (Supervisor):**
   * **Role:** Central coordination and benchmarking controller.
   * **Function:**  
     * Spawns trash deterministically  
     * Monitors simulation state  
     * Runs auctions  
     * Assigns tasks  
     * Logs all task allocation decisions  

2. **The Collectors (TurtleBot3 Burger):**
   * **Role:** Worker robots under evaluation.
   * **Function:**  
     * Receive task coordinates only  
     * Plan paths using A*  
     * Execute navigation and obstacle avoidance  
     * Log execution metrics  

3. **The Human Agent:**
   * **Role:** Dynamic obstacle.
   * **Function:**  
     * Patrols a fixed route  
     * Introduces non-deterministic motion  
     * Stress-tests collision avoidance consistency across strategies  

---

## 1. Task Allocation Benchmark (Auction System)

**Managed by:** `controllers/auctioneer/auctioneer.py`

The auction mechanism is implemented as a **Single-Item Auction Protocol**, allowing clean isolation of allocation logic from navigation and sensing.

### Auction Lifecycle

1. **Announcement**  
   The Auctioneer broadcasts an `auction_start` message with a trash location.

2. **Bidding**  
   Idle Collectors compute their path cost (Euclidean distance) and submit a `bid`.

3. **Winner Selection**  
   After a fixed timeout (2.0s), the Auctioneer selects the lowest bid.

4. **Assignment**  
   The winner receives an `assign_task` command and queues the task.

5. **Task Chaining**  
   Robots may queue multiple assignments, preventing idle gaps.

---

## Auction Strategies Under Test

All strategies are interchangeable via configuration, enabling **direct benchmarking**.

### A. Sequential Allocation (`"sequential"`)

**Purpose:** Greedy baseline for auction-based coordination.

* Tasks are auctioned strictly in ID (FIFO) order.
* Ignores robot distribution and global efficiency.

```text
[ Task List (Sorted by ID) ]
┌─────────┬─────────┬─────────┐
│ Task_00 │ Task_01 │ Task_02 │ ...
└────┬────┴─────────┴─────────┘
     │
     ▼
( Selects First )
     │
┌────▼────────────┐
│   AUCTIONEER    │ ── "Who wants Task_00?" ──▶ [ Collectors ]
└─────────────────┘
````

---

### B. Nearest Task Allocation (`"nearest_task"`)

**Purpose:** Primary optimisation strategy under evaluation.

* Computes distance from **every idle robot** to **every available task**.
* Selects the task that minimises overall fleet travel cost.

```text
    Task_A                      Task_B
      │                           │
(5m away)                   (1m away)
      │                           │
┌─────▼─────┐               ┌─────▼─────┐
│ Collector │               │ Collector │
│    #1     │               │    #2     │
└───────────┘               └───────────┘
      ▲                           ▲
      └────────────┬─────────────┘
                   │
            (Compare Distances)
                   │
        ┌──────────▼───────────┐
        │      AUCTIONEER       │
        │  Selects Task_B next  │ ── "Who wants Task_B?" ──▶
        └──────────────────────┘
```

---

### C. Random Allocation (`"random"`)

**Purpose:** Stress-test and control strategy.

* Randomly selects a task from the pool.
* Useful for robustness testing and null-hypothesis comparison.

```text
[ Task Pool ]
{ Task_05, Task_02, Task_09, Task_01 }
       │
       ▼
 (Random Roll) ──▶ Picks Task_09
       │
┌──────▼─────────┐
│   AUCTIONEER   │ ── "Who wants Task_09?" ──▶ [ Collectors ]
└────────────────┘
```

---

## 2. Navigation Stack (Controlled Variable)

**Managed by:** `controllers/lib_shared/navigation.py`

Navigation is intentionally **identical across all experiments** to isolate coordination effects.

### Global Planner (A*)

* Binary occupancy grid (`map_module.py`)
* Produces optimal grid-based paths
* Smoothed into world-space waypoints

### Local Planner (Reactive Avoidance)

* Lightweight avoidance (`obstacle_avoidance.py`)
* LiDAR-based proximity detection
* Slows or rotates in place when obstacles < 0.25m
* DWA intentionally removed in v2.0 to improve repeatability

---

## Installation & Usage

### Prerequisites

* **Webots R2025a** (or compatible)
* **Python 3.8+**

### Dependencies

Only the Python Standard Library and Webots API are used.

* `controller`
* `math`, `json`, `sys`, `os`, `datetime`, `random`
* `collections`, `heapq`, `dataclasses`, `typing`

---

## Running the Benchmark

1. **Clone the repository**

   ```bash
   git clone https://github.com/ZahinMai/Clean-Up-Crew.git
   ```

2. **Open Webots**

   * Load: `worlds/cafetria.wbt`

3. **Select Benchmark Mode**

   * In the scene tree, select the `auctioneer` node
   * Edit `customData`:

     * `SETUP:BASELINE`
     * `SETUP:SWARM`
     * `SETUP:AUCTION`

4. **Press Play**

   * Trash spawning, task allocation, and logging begin immediately

---

## Configuration

### Auction Strategy Selection

Edit `controllers/lib_shared/CONFIG.py`:

```python
# Options: "sequential", "nearest_task", "random"
auction_strategy = "nearest_task"
```

---

## Logging & Experimental Output

Each run automatically generates timestamped logs to support **quantitative and qualitative analysis**.

### Auctioneer Logs

* Location: `controllers/auctioneer/logs/`
* Records:

  * Auction starts
  * Bids received
  * Winner selection
  * Task completion

### Collector Logs

* Location: `controllers/collector/logs/`
* Records:

  * FSM transitions (`IDLE → NAVIGATING`)
  * Path length and waypoint count
  * ASCII occupancy grid visualisations

Example filenames:

```text
auction_output_20251211_224745.md
nav_output_20251211_224745.md
```

---

## File Structure

```text
Clean-Up-Crew/
├── controllers/
│   ├── auctioneer/              # Central coordinator & benchmark controller
│   │   └── auctioneer.py
│   ├── collector/               # Worker robot FSM
│   │   └── collector.py
│   ├── Human_agent/             # Dynamic obstacle
│   ├── lib_shared/              # Shared infrastructure
│   │   ├── communication.py
│   │   ├── global_planner.py
│   │   ├── navigation.py
│   │   ├── obstacle_avoidance.py
│   │   └── map_module.py
│   └── archive/                 # Deprecated v1.0 components
│       └── spotter.py
└── worlds/
    └── cafetria.wbt
```

---

## Contributors

* **Zahin Maisa:** System architecture, benchmarking design, auctioneer, A* navigation, collector FSM
* **Abdullateef Vahora:** Legacy vision system (v1.0)
* **Ajinkya:** World design, dynamic obstacle, human agent
* **Kunal:** Communication protocol

---

## License

This project is licensed under the **MIT License**.

