# Project Contributors

### **Zahin Maisa**
* **System Architecture:** Designed the version 2.0 Multi-Agent System (MAS) architecture, shifting from a Spotter-based bottleneck to a Supervisor-controlled auction system.
* **Navigation Stack:** Developed the `AStarPlanner` global path planner with path smoothing and Bresenham’s Line Algorithm for line-of-sight checks.
* **Collector Logic:** Authored the `collector.py` Finite State Machine (FSM), managing states from `IDLE` to `NAVIGATING` and handling task queueing.
* **Auctioneer:** Implemented the `auctioneer.py` supervisor, including the "Sequential," "Nearest-Task," and "Random" task allocation strategies.
* **Infrastructure:** Developed the `OccupancyGrid` mapping module and the `dual_logger.py` utility for automated experimental reporting.

### **Abdullateef Vahora**
* **Vision System:** Authored `vision.py`, implementing a DFS flood-fill algorithm for cluster-based blue pixel detection to identify trash.
* **Coverage Planning:** Developed the `CoveragePlanner` in `coverage.py`, which decomposes the grid into zones for lawnmower and zigzag patrol patterns.
* **Experimental Setups:** Authored `collector_coverage.py` to handle logic for the **BASELINE** and **SWARM** benchmarking modes.
* **Legacy Components:** Developed the original version 1.0 `spotter.py` and `supervisor.py` controllers (now archived).

### **Kunal Khose**
* **Communication Module:** Authored the `communication.py` library, handling JSON-based message passing between the Auctioneer and Collectors using Webots Emitters and Receivers.
* **Documentation:** Led the development of the formal report structure and Overleaf documentation.
* **Integration:** Assisted in integrating communication protocols into the Spotter and Collector agents.

### **Ajinkya Patil**
* **Local Planning:** Implemented the Dynamic Window Approach (`DWA`) in `local_planner.py` for reactive obstacle avoidance.
* **World Design:** Built and refined the `cafetria.wbt` environment, including the table grid layout and obstacle placement.
* **Human Agent:** Developed the `human.py` controller to act as a dynamic obstacle for stress-testing robot navigation.
* **Media:** Produced final video demonstrations and assisted with plotting experimental results (to be added to repo)

> Generated based on the project logs, commit history, and source code headers.
