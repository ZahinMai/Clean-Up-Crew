# Cafeteria Clean-Up-Crew: Auction-Based Multi-Robot Coordination
**A Webots simulation framework implementing a "Spotter-Collector" architecture for efficient automated cleaning.**

## Overview

This project simulates a cafeteria environment where a coordinated team of robots identifies and removes trash. Unlike brute-force swarm approaches, this system utilizes **role specialisation** and an **auction-based task allocation** protocol to minimize redundant travel and maximize efficiency.

### The Team (v2.0 Architecture)

1.  **The Auctioneer (Supervisor):**
    * **Role:** Replaces the physical "Spotter" robot from v1.0.
    * **Function:** Spawns trash, monitors simulation state, and broadcasts auctions to workers. It serves as the central coordination node.
2.  **The Collectors (TurtleBot3 Burger):**
    * **Role:** Worker robots.
    * **Function:** "Blind" agents that receive task coordinates via auctions. They navigate using a global A* planner and local reactive avoidance.
3.  **The Human Agent:**
    * **Role:** Dynamic Obstacle.
    * **Function:** Patrols a fixed route to test the Collectors' collision avoidance capabilities.

---

## System Architecture

The project has evolved into a robust Multi-Agent System (MAS) focusing on task allocation efficiency and navigation stability.

### 1. Task Allocation (The Auction)
**Managed by:** `controllers/auctioneer/auctioneer.py`

The system implements a **Single-Item Auction** protocol:
1.  **Announcement:** The Supervisor broadcasts an `auction_start` message with the location of a trash item.
2.  **Bidding:** Idle Collectors calculate the path cost (Euclidean/A* distance) to the target and submit a `bid`.
3.  **Winner Selection:** The Supervisor waits for a timeout (2.0s), selects the lowest bidder, and sends an `assign_task` command.
4.  **Task Chaining:** Robots can queue tasks if assigned multiple targets, preventing idle time.

#### Strategies & Logic
You can toggle the allocation logic in `controllers/lib_shared/CONFIG.py`.

**A. Sequential Allocation (`"sequential"`)**
* **Logic:** Auctions tasks in ID order (FIFO). This is the default greedy approach.
```text
      [ Task List (Sorted by ID) ]
      ┌─────────┬─────────┬─────────┐
      │ Task_00 │ Task_01 │ Task_02 │ ...
      └────┬────┴─────────┴─────────┘
           │
           ▼
    ( Selects First )
           │
   ┌───────▼───────┐
   │  AUCTIONEER   │  ── "Who wants Task_00?" ──▶  [ Collectors ]
   └───────────────┘
````

**B. Nearest Task Allocation (`"nearest_task"`)**

  * **Logic:** The Auctioneer calculates the distance from **every idle robot** to **every available task** and selects the task that minimizes travel time for the fleet.

<!-- end list -->

```text
        Task_A                        Task_B
          │                             │
    (5m away)                     (1m away)
          │                             │
    ┌─────▼─────┐                 ┌─────▼─────┐
    │ Collector │                 │ Collector │
    │    #1     │                 │    #2     │
    └───────────┘                 └───────────┘
          ▲                             ▲
          └──────────────┬──────────────┘
                         │
                  (Compares Distances)
                         │
             ┌───────────▼───────────┐
             │      AUCTIONEER       │ ── "Task_B is closest to a bot."
             │  Selects Task_B next  │ ── "Who wants Task_B?" ──▶
             └───────────────────────┘
```

**C. Random Allocation (`"random"`)**

  * **Logic:** Selects a random task from the pool to load-balance or test robustness.

<!-- end list -->

```text
     [ Task Pool ]
    { Task_05, Task_02, Task_09, Task_01 }
           │
           ▼
    (Random Roll) ──▶ Picks Task_09
           │
   ┌───────▼───────┐
   │  AUCTIONEER   │  ── "Who wants Task_09?" ──▶  [ Collectors ]
   └───────────────┘
```

### 2\. Navigation Stack

**Managed by:** `controllers/lib_shared/navigation.py`

  * \**Global Planner (A*):\*\*
      * Uses a binary **Occupancy Grid** (`map_module.py`) to represent the cafeteria.
      * Plans an optimal path of grid cells, then smooths them into world-space waypoints.
  * **Local Planner (Reactive Avoidance):**
      * *Note: DWA was removed from Collectors in v2.0 for stability.*
      * Uses **Simple Avoidance** (`obstacle_avoidance.py`): A lightweight controller that slows down or turns in place when LiDAR detects obstacles within a critical radius (\<0.25m).

-----

## Installation & Usage

### Prerequisites

  * **Webots R2025a** (or compatible)
  * **Python 3.8+**

### Running the Simulation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/ZahinMai/Clean-Up-Crew.git](https://github.com/ZahinMai/Clean-Up-Crew.git)
    ```
2.  **Launch Webots:**
    Open the world file: `worlds/cafetria.wbt`.
3.  **Start:**
    Press **Play**. The Supervisor will spawn trash and begin the auction cycle immediately.

### Configuration

To change the auction strategy, edit `controllers/lib_shared/CONFIG.py`:

```python
# Options: "sequential", "nearest_task", "random"
auction_strategy = "nearest_task"
```

-----

## File Structure

```text
Clean-Up-Crew/
├── controllers/
│   ├── auctioneer/             # Supervisor logic (New central coordinator)
│   │   └── auctioneer.py       # Main spawning & auction logic
│   ├── collector/              # Worker robot logic
│   │   └── collector.py        # FSM: IDLE <-> NAVIGATING
│   ├── Human_agent/            # Dynamic obstacle logic
│   ├── lib_shared/             # SHARED LIBRARIES
│   │   ├── communication.py    # JSON messaging wrapper
│   │   ├── global_planner.py   # A* Implementation
│   │   ├── navigation.py       # Navigation Manager
│   │   ├── obstacle_avoidance.py # Simple Reactive Dodge
│   │   └── map_module.py       # Occupancy Grid
│   └── archive/                # Deprecated components
│       └── spotter.py          # Legacy visual search robot (v1.0)
└── worlds/
    └── cafetria.wbt            # Simulation Environment
```

-----

## Contributors

  * **Zahin Maisa:** Project Lead, Architecture, A\* Navigation, Collector FSM.
  * **Abdullateef Vahora:** Supervisor/Auctioneer Logic, Legacy Vision System.
  * **Ajinkya:** World Design, Dynamic Obstacle (Human Agent).
  * **Kunal:** Communication Protocol, Documentation.

-----

## License

This project is licensed under the **MIT License**.
