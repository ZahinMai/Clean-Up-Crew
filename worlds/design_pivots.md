# Design Pivots: Architecture Retrospective
**Author:** Zahin Maisa  
**Project:** Clean-Up-Crew Multi-Agent System  
**Date:** December 2025

---

## Overview
During the testing phase of the "Spotter-Collector" architecture, several critical limitations were identified in the coordination and execution logic. This file outlines the key design pivots required to transition from the current prototype to a robust, scalable Multi-Agent System (MAS).

---

## 1. Task Allocation Strategy
### From **Sequential Greedy** to **Combinatorial Optimization**

* **The Issue:** The current auctioneer spawns all trash items, then iterates through tasks one by one, assigning them to the lowest bidder when either Collector is IDLE. This results in Collectors driving over unassigned trash, but the task not registering as complete.
* **The Pivot:** 

## 2. Robot Autonomy Model
### From **Rigid FSM** to **Opportunistic Subsumption**

* **The Issue:** The Collector agents operate on a strict Finite State Machine (`IDLE` $\leftrightarrow$ `NAVIGATING`). This created a "Tunnel Vision" effect where robots would physically drive past unassigned trash but ignore it because their internal state was locked to a specific coordinate goal.
* **The Pivot:** Adopt a **Layered/Subsumption Architecture**.
    * **New Design:** Navigation to the assigned goal becomes a high-level behavior, but it can be *subsumed* (overridden) by a lower-level "Reactive Collection" behavior. If the robot's sensors detect *any* trash within a critical radius during transit, it should pause the navigation plan, collect the object, and then resume.

## 3. Temporal Coordination
### From **Synchronous Stop-and-Go** to **Asynchronous Chaining**

* **The Issue:** Currently, agents reset to an `IDLE` state after every task and wait for the central Auctioneer to initiate a new round. This "Stop-and-Go" latency prevents smooth operation and introduces significant communication overhead.
* **The Pivot:** Implement **Task Chaining (Queueing)**.
    * **New Design:** Robots should maintain a local queue of tasks. When a robot is nearing completion of Task A, it should participate in auctions for Task B *in parallel*, allowing for seamless transitions and continuous movement without idle waiting periods.

## 4. Simulation Fidelity & Validation
### From **Event-Driven Logic** to **State-Based Physics**

* **The Issue:** The Supervisor's despawn logic is triggered solely by the `collected` message from a robot. This decouples the "game logic" from the "physical simulation," leading to scenarios where a robot might accidentally push trash away without collecting it, or "collect" trash that it isn't actually touching if the navigation tolerance is loose.
* **The Pivot:** Enforce **Proximity-Based Validation**.
    * **New Design:** The Supervisor should monitor the simulation state (Physics Step) directly. Trash should only despawn if a valid `Collector` entity is physically within $0.1m$ of the object, regardless of the robot's internal software state or active task ID.

---
*This document serves as a roadmap for the `v2.0` architectural refactor.*