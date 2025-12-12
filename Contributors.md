Based on a thorough analysis of the provided source code headers, project logs, and file metadata, here is a detailed `CONTRIBUTORS.md` file.

This breakdown attributes ownership based on explicit `@author` tags found in the code, commit history described in the logs, and the specific implementation details within each file.

***

# Contributors

## **Zahin Maisa**
**Role:** Lead Architect & Navigation Developer

Zahin was responsible for the core system architecture, the central navigation stack, and the finite state machines for the primary robots.

* **System Architecture**: Designed the "Spotter-Collector" and later "Auctioneer-Collector" architectures, including the `Collector` FSM.
* **Global Navigation**: Implemented the A* pathfinding algorithm (`AStarPlanner`) and the `OccupancyGrid` mapping module (`map_module.py`).
* **Navigation Management**: Created the `Navigator` class which integrates global planning with local collision avoidance.
* **Obstacle Avoidance**: Wrote the `SimpleAvoidance` logic (used by Collectors in the final build) to replace the unstable DWA implementation.
* **Task Allocation**: Developed the `Auctioneer` supervisor controller to manage trash spawning, task bidding, and assignment.
* **Tooling**: Created the `DualLogger` to capture console output into Markdown reports for analysis.

**Key Files:**
* `controllers/auctioneer/auctioneer.py`
* `controllers/collector/collector.py`
* `controllers/lib_shared/global_planner.py`
* `controllers/lib_shared/map_module.py`
* `controllers/lib_shared/navigation.py`
* `controllers/lib_shared/obstacle_avoidance.py`

---

## **Abdullateef Vahora**
**Role:** Computer Vision & Coverage Specialist

Abdullateef focused on the sensing and searching capabilities, specifically regarding how robots identify trash and cover ground efficiently.

* **Computer Vision**: Implemented `vision.py` using pixel-clustering algorithms (DFS flood-fill) to detect "blue blob" trash items.
* **Coverage Planning**: Developed `coverage.py`, which decomposes the map into zones and generates "lawnmower" search patterns for the Spotter.
* **Legacy Components**: Built the original `Spotter` controller and `Supervisor` logic for the initial experimental setup.
* **Coverage Controller**: Implemented the specific logic for the coverage-based experimental setup in `collector_coverage.py`.

**Key Files:**
* `controllers/lib_shared/vision.py`
* `controllers/lib_shared/coverage.py`
* `controllers/collector/collector_coverage.py`
* `archive/spotter.py`
* `archive/supervisor.py`

---

## **Ajinkya**
**Role:** Local Planning & Simulation Environment

Ajinkya was responsible for the physical simulation environment and advanced local path planning algorithms.

* **Local Planner (DWA)**: Authored `local_planner.py`, implementing the **Dynamic Window Approach** to calculate safe velocity trajectories `(v, w)` based on LiDAR data.
* **World Design**: Designed and built the Webots simulation world (`cafetria.wbt`), including walls, furniture layouts, and lighting.
* **Human Agent**: Developed the `Human_agent` controller, a dynamic obstacle that patrols the cafeteria to test robot avoidance capabilities.

**Key Files:**
* `controllers/lib_shared/local_planner.py`
* `controllers/Human_agent/human_agent.py`
* `worlds/cafetria.wbt`

---

## **Kunal**
**Role:** Communication & Protocols

Kunal focused on the multi-robot communication infrastructure required for the auction system.

* **Communication Library**: Developed `communication.py`, providing a high-level wrapper around Webots' `Emitter` and `Receiver` devices. This library standardizes JSON message passing for events like `auction_start`, `bid`, and `assign_task`.
* **Auction Integration**: Integrated the communication protocols into the legacy `spotter.py` to enable initial auction testing.

**Key Files:**
* `controllers/lib_shared/communication.py`