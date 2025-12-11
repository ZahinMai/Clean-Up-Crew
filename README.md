# Cafeteria Clean-Up-Crew: Auction-Based Multi-Robot Coordination
**A Webots simulation framework implementing a "Spotter-Collector" architecture for efficient automated cleaning.**

## Overview

This project simulates a cafeteria environment where a coordinated team of robots identifies and removes trash. Unlike brute-force swarm approaches, this system utilizes **role specialisation** and an **auction-based task allocation** protocol to minimize redundant travel and maximize efficiency.

### The Team

1.  **The Spotter (e-puck):** A robot that patrols the environment using a coverage algorithm. It detects trash using computer vision and "auctions" the cleaning task to the "Collector" fleet.
2.  **The Collectors (TurtleBot3 Burger):** "Blind" worker robots that bid on tasks based on their distance and current state. The winner plans it's path to the target using A* (global_planer) and avoids the human (dynamic obstacle) using DWA (local_planner)
3.  **The Supervisor:** Manages the simulation state, spawns trash objects (blue spheres), and tracks successful collections.
4.  **Human Agent:** A dynamic obstacle that moves between waypoints to test robot avoidance capabilities.

-----

## System Architecture

The project is built on a modular architecture separating high-level logic from low-level control.

### 1\. Communication & Task Allocation

Implemented in `lib_shared/communication.py`.

  * **Protocol:** JSON-based messaging via Webots Emitter/Receiver nodes.
  * **Auction Process:**
    1.  **Discovery:** Spotter detects trash and broadcasts an `auction_start` event with coordinates.
    2.  **Bidding:** Idle Collectors calculate the A\* path cost to the target and reply with a `bid`.
    3.  **Assignment:** The Spotter (or testing agent) assigns the task to the lowest bidder.
    4.  **Execution:** The winning Collector transitions to `NAVIGATING` state.

### 2\. Navigation Stack

  * **Global Planner (`AStarPlanner`):**
      * Uses an Occupancy Grid (`map_module.py`) to represent the cafeteria.
      * Calculates the optimal path using A\* with Euclidean distance heuristics.
      * Includes path smoothing to remove unnecessary waypoints.
  * **Local Planner (`DWA`):**
      * Implements the **Dynamic Window Approach** to avoid dynamic obstacles (like the Human Agent) while tracking the global path.
      * Calculates safe linear and angular velocities based on LiDAR point clouds.

### 3\. Perception

  * **Vision (`vision.py`):**
      * Uses a DFS (Depth-First Search) flood-fill algorithm on camera images to cluster pixels.
      * Identifies objects based on RGB thresholds (specifically filtering for blue "trash" objects).
  * **Coverage (`coverage.py`):**
      * Decomposes the map into rectangular zones.
      * Generates "lawnmower" patterns to ensure the Spotter visually covers the entire accessible floor area.

-----

## Installation & Usage

### Prerequisites

  * **Webots R2025a** (or compatible version)
  * **Python 3.8+** (Standard libraries used: `math`, `json`, `heapq`, `collections`)

### Running the Simulation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/ZahinMai/Clean-Up-Crew.git
    ```
2.  **Launch Webots:**
    Open the world file: `worlds/cafetria.wbt`.
3.  **Start the Simulation:**
    Press the **Play** button in the Webots interface.
      * The **Supervisor** will spawn 10 blue trash objects.
      * The **Spotter** will begin its patrol pattern.
      * **Collectors** will idle until receiving tasks.

### Running Automated Tests

To verify the auction logic and navigation without waiting for the full simulation loop:

1.  Select the **Spotter** robot in the Webots scene tree.
2.  Change the `controller` field from `spotter` to `test_spotter_comm`.
3.  Save and Reload.
4.  The system will run through 5 scenarios (Sanity Check, Static Nav, Dynamic Avoidance, Multi-Agent Competition).
5.  Results are logged to the console and `logs/` directory.

-----

## File Structure

```text
Clean-Up-Crew/
├── controllers/
│   ├── collector/              # Logic for worker robots
│   │   ├── collector.py        # Main FSM (IDLE <-> NAVIGATING)
│   │   └── logs/               # Execution reports
│   ├── spotter/                # Logic for the sensing robot
│   │   └── spotter.py          # Vision and Coverage integration
│   ├── supervisor/             # Game master logic
│   │   └── supervisor.py       # Spawns trash, tracks score
│   ├── Human_agent/            # Dynamic obstacle logic
│   ├── test_spotter_comm/      # Unit testing for auctions
│   └── lib_shared/             # CORE LIBRARIES
│       ├── communication.py    # JSON messaging wrapper
│       ├── coverage.py         # Zone decomposition planner
│       ├── global_planner.py   # A* implementation
│       ├── local_planner.py    # DWA implementation
│       ├── map_module.py       # Grid representation
│       ├── dual_logger.py      # Logging utility
│       └── vision.py           # Pixel processing
└── worlds/
    ├── cafetria.wbt            # Main simulation environment
    └── .cafetria.wbproj        # Project configuration
```

-----

## Contributors

  * **Zahin Maisa:** Architecture, Global Navigation (A\*), Testing Framework.
  * **Abdullateef Vahora:** Spotter Logic, Computer Vision, Coverage Planning.
  * **Ajinkya:** Local Path Planning (DWA).
  * **Kunal:** Communication Protocol.

-----

## License

This project is licensed under the **MIT License**. See [LICENSE](https://www.google.com/search?q=LICENSE) for details.
