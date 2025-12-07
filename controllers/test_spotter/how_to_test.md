# Testing Auction-Bid Task Allocation

## Overview
The `SpotterTester` controller issues auction requests for trash collection. The robots must:
1. Receive the task via communication.
2. Calculate path cost (A* distance).
3. Bid on the task.
4. Winner navigates to target while avoiding dynamic obstacles (humans).

## Setup
1. Open the Webots world file.
2. Ensure the `SpotterTester` supervisor is present in the scene tree.
3. Ensure at least **2 Collector Bots** are placed in the arena.
4. Ensure the **Human Worker** (dynamic obstacle) is active/moving.

## How to Run the Tests
The system runs in **Automated Mode** by default.

1. **Press Play** in Webots.
2. Wait 5 seconds for system initialization.
3. The `SpotterTester` will automatically cycle through 5 scenarios.
4. Watch the **Console** for real-time logs.
5. Watch the **3D View** for robot behavior.

### Manual Override
If you want to trigger tests one by one:
* Press **`R`** to Reset.
* Press **`A`** to manually trigger the next Auction/Task.

## Test Cases & Success Criteria

### 1. Communication & Bidding
* **Trigger:** Any task.
* **Verify:** Check console logs. You should see multiple `-> BID [ID]: [COST]` lines followed by `Lowest path cost ...`.
* **Pass:** The bot with the lowest reported cost is assigned the task.
* **Fail:** No bids received, or a bot with a higher cost (further away) wins.

### 2. A* Static Navigation
* **Trigger:** Task "Navigation (Behind Tables)".
* **Visual Check:** The bot should move *around* the table/chairs.
* **Pass:** Bot arrives at the target without touching static furniture.
* **Fail:** Bot gets stuck on a table leg or tries to drive through a wall.

### 3. Dynamic Obstacle Avoidance
* **Trigger:** Task "Human Avoidance Zone" (Task #3).
* **Visual Check:** Wait for the bot to cross the center. If a human walks in front:
    * **Pass:** Bot stops, waits, or recalculates a path around the human.
    * **Fail:** Bot collides with the human or pushes them.

### 4. Multi-Agent Efficiency
* **Trigger:** Tasks #4 and #5.
* **Verify:** These tasks occur back-to-back.
* **Pass:** Two *different* robots should handle these tasks (since the first winner is now "Busy").
* **Fail:** The same robot tries to win both, or the second task is ignored.

## Log Analysis
After the test completes (Phase 99), a file named `test_run_output.txt` is generated.

**Search the log for:**
* `COMPLETED task`: Counts total successes.
* `Success: 100.0%`: Ideal outcome.
* `Parse error`: Indicates corrupted communication packets.

---
**Author:** Zahin