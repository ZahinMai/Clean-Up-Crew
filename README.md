# Clean-Up-Crew

## Coordination and Roles in Multi-Robot Cleaning Performance
This project will investigate how coordination and role specialisation affect performance in multi-robot
tasks. In theory, adding more robots should improve performance. In practice, robots can waste effort
through path overlap, item contention, and collisions. Coordination promises efficiency, but also
introduces overhead. One study shows how cleaning success rises by only about 6.8% when moving from
a single-robot baseline to a two-robot team (Patil, Banavar and Narayanan, 2023).

As the exact tipping point between benefit and overhead is context-dependent, we will explore these ideas
in a simple, structured simulation. This will be a webots cafeteria cleaning task where robots will avoid
obstacles and pick up scattered rubbish (detected using color-based vision). We will compare three
setups under identical conditions:

• Setup 1: Single general-purpose robot (baseline)
• Setup 2: Three independent general-purpose robots (no communication)
• Setup 3: One Spotter + Two Collectors (coordinated team with communication)

These configurations will allow us to test the below hypotheses:
- H1 (Scaling): Increasing from one to three independent robots reduces total cleaning time but with
diminishing returns.
- H2 (Coordination Advantage): The Spotter-Collector team completes tasks faster with fewer
redundant pickups than three independent robots.
- H3 (Efficiency): The coordinated team covers less total distance per robot while maintaining or
improving completion time.

Whilst we will reimplement existing algorithms for effective navigation, our unique contribution is in
empirically testing not only how performance scales with more robots, but also how coordination affects
performance in a simple, controlled environmen

# Team Task Split

## Zahin
**Mapping, Planning and Collector controller**

- [x]  Build 2D occupancy grid
- [x]  Path planning with A*
- [ ]  Text-based visualisations for debugging
- [x]  Collector navigation
- [ ]  Implement FSM (IDLE → GETTING_TASK → GOING_TO_TRASH → GOING_TO_BIN)
- [ ]  Emit `im_idle` status

---
## **Abdullateef**
**Perception, CPP and Spotter controller**

- [x]  Implement color-blob detection with camera
- [ ]  Output trash coordinates
- [x]  Implement Boustrophedon (lawn-mower) pattern

---

## **Ajinkya**
### **Environment & Obstacle Avoidance **
- [x]  Build cafeteria world (walls, tables, bin zone)
- [x]  Add dynamic obstacle patrol bot
- [ ]  Implement DWA using lidar (to be completed) that outputs safe (v, ω) (to be completed)
- [ ]  Integrate A* with DWA in Collector controller. Test full collector navigation loop.

## **Kunal**

**Communication**
- [ ]  Implement Emitter + Receiver wrappers, define message formats
- [ ]  Emit `trash_found(x, y)` via comms API
- [ ]  Listen for `trash_found` and `im_idle`
- [ ]  Allocate tasks & Emit `go_to_task` to collectors

---

## **Zekai**
### Experiment supervisor controller
- [ ]  Detect trash deposit (TouchSensor)
- [ ]  Despawn trash object
- [ ]  Start/stop experiment timer
- [ ]  Log score + key events, Record final results
