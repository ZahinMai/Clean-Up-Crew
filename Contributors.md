# Contribution Report (Generated to prevent bias)

**Date:** December 07, 2025  
**Repository:** Clean-Up-Crew  
**Based on:** Code headers, metadata, and commit history.

---

### **Zahin Maisa** (Project Lead & Core Infrastructure)
* **Role:** Architect, Repository Maintainer, Core Developer
* **Key Contributions:**
    * **Architecture:** Designed the base project structure, including the `Collector` agent state machine (IDLE/NAVIGATING).
    * **Global Navigation:** Implemented `AStarPlanner` for grid-based pathfinding and the `OccupancyGrid` mapping module.
    * **Testing & Tooling:** Created the `SpotterTester` automation framework and the `DualLogger` for generating markdown reports from console output.
    * **Maintenance:** Managed repository setup, `.gitignore`, and licensing.

### **Abdullateef Vahora** (Perception & Coverage)
* **Role:** Spotter Agent Developer
* **Key Contributions:**
    * **Spotter Logic:** Developed the `SpotterController`, responsible for high-level task assignment.
    * **Computer Vision:** Implemented `vision.py` to detect trash objects using pixel clustering algorithms (DFS flood-fill).
    * **Coverage Planning:** Created `spotter_coverage.py` to decompose the map into zones and generate efficient "lawnmower" search paths.

### **Ajinkya** (Navigation Algorithms)
* **Role:** Local Path Planning Developer
* **Key Contributions:**
    * **Obstacle Avoidance:** Authored the `DWA` (Dynamic Window Approach) module in `local_planner.py`. This logic allows robots to calculate safe velocities to avoid dynamic obstacles while moving toward a target.

### **Kunal** (Networking)
* **Role:** Communications Developer
* **Key Contributions:**
    * **Communication Protocol:** Developed the `Communication` library in `communication.py`, abstracting Webots' Emitter/Receiver nodes into a simplified message passing interface for JSON packets.

---

## Recent Activity
* **Feature Integration:** Recent merges include the integration of the `spotter-vision` feature branch by Abdullateef Vahora.
* **Infrastructure Updates:** Recent commits by Zahin Maisa focused on enhancing navigation modules and stabilising the logging framework.
* **Current State:** The project is active, with the latest version addressing logging errors to ensure reliable test reporting.