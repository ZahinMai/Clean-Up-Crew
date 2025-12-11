# PROJECT DISCUSSIONS

### w/c 03/11/2025

### Discussed

- Agreed core setup: **Webots 2024a**, **Python controllers**, repo structure:
    - `/controllers/{spotter, collector, supervisor, shared_libs}` (nav_lib.py, comms_lib.py, vision_lib.py, fsm_lib.py), `/worlds/cafeteria.wbt`, `/logs/run_metrics.csv`.
- First formal **Meeting 1**: read assignment (CleanUpCrew.pdf), defined algorithms/roles:
    - Ajinkya – **DWA** local navigation with range sensors.
    - Zahin – *A/D** grid planner + dynamic replanning.
    - Abdul – **Markov localisation** with noisy odometry + vision-based belief grid.
    - Zekai – **Bellman update** in FSM (GO_TO_TRASH dynamic cost).
    - Kunal – **Auction-based task allocation** via Emitter/Receiver JSON.
- Pre-reqs: Webots, Notion, GitLab; this week’s target: **feasibility study** of each part.
- Agreed to use a **Notion workspace** (with Meeting 1 notes + Abdul’s “balanced split” tasks).
- Early robot/world discussion: e-puck, Tiago, cafeteria world.

### Progress Status

- GitLab repo `clean-up-crew` created with agreed folder structure.
- Ajinkya creates initial **cafeteria.wbt** and shares it.
- Notion workspace active with Meeting 1 notes and algorithm assignments; plan to get an end-to-end MVP using **library code first**, then swap to custom implementations.
- Team confirms basic tool usage (Notion, GitLab); contribution tracking started.

### Blockers & Dependencies

- Several members hadn’t fully read the assignment initially; robot choice not locked in.
- No integrated system yet, only **individual feasibility work**.

### Action Items

- [ ]  **Zahin** – Set up/maintain repo + Notion; start A*/D* feasibility and nav planning.
- [ ]  **Ajinkya** – Refine cafeteria world; prototype DWA.
- [ ]  **Abdul** – Start coverage/localisation MVP once world stable; document work split.
- [ ]  **Kunal** – Begin auction–based task allocation design.
- [ ]  **Zekai** – Explore Bellman update and negative rewards R(s,a).
- [ ]  **Team** – Complete individual feasibility studies using libraries.

---

### w/c 10/11/2025

### Discussed

- Check progress on navigation, coverage, RL, etc.; clarify if Zahin owns **path planning only** or **navigation + planning**.
- Support for **Zekai** with RL/Bellman; reaffirm need for an **end-to-end MVP** that was already late.

### Progress Status

- Zahin’s **A**path planner works; navigation controller “awful” and still under development.
- Abdul continues coverage/localisation work, planning a focused robotics day.
- Zekai struggles with negative reward design R(s,a) for Bellman update.
- Z emphasises committing even non-working code to show continuous work.

### Blockers & Dependencies

- Navigation not yet cleanly integrated with A*; no controller using A* + DWA.
- RL/FSM piece unclear and heavy for Zekai.
- MVP still missing; the team is behind schedule.

### Action Items

- [ ]  **Zahin** – Improve navigation; integrate with A*; commit frequently.
- [ ]  **Abdul** – Progress coverage/localisation and assist debugging if others are stuck.
- [ ]  **Zekai** – Push code for review; refine R(s,a).
- [ ]  **Team** – Align on a new realistic MVP date; hold a short clarifying call.

---

### w/c 17/11/2025

### Discussed

- Acknowledged there are only ~2 weeks for **full report + demo**.
- Need to start **gluing parts together** (integration focus).
- Decision to move from **GitLab to GitHub** so Zekai can participate.
- Plan for a **live collab session** to teach Git/Webots and integrate modules.

### Progress Status

- Abdul expects his components “fully done” by mid-week.
- Zahin & Ajinkya have local/global planners from earlier TurtleBot work; arms added to robots, later recognised as overkill since actual picking-up is not strictly needed.
- GitHub migration started; emails collected; everyone added.
- Z applies for **EC** (due to dad’s stroke), suggests possible extra week; team aware of time pressure.

### Blockers & Dependencies

- A* navigator calibration broken; Zahin needs help with mapping vs world alignment.
- Skill gaps in Git/Webots slow integration.
- Individual parts still not integrated; system-level behaviour unknown.

### Action Items

- [ ]  **Zahin** – Fix A* calibration; complete GitHub migration; keep GitLab screenshot as evidence.
- [ ]  **Abdul** – Finish coverage algo; help debug nav; propose an integration call.
- [ ]  **Ajinkya** – Confirm world/collector compatibility with A*/coverage; assist tuning.
- [ ]  **Kunal & Zekai** – Get on GitHub, push work, join integration call.
- [ ]  **Team** – Run ≥2 integration session and shift focus from solo work to team integration.

---

### w/c 24/11/2025

### Discussed

- Clear status snapshot shared by Zahin:
    - **Zahin (Collector)**:
        - Done: 2D occupancy grid, A* global planner (waypoints), A* visualisation.
        - Pending: FSM (IDLE → GETTING_TASK → GOING_TO_TRASH → GOING_TO_BIN), `im_idle` status, integrate A* with DWA, full collector navigation tests.
    - **Abdul (Spotter)**:
        - Done: colour-blob detection, start on Boustrophedon pattern.
        - Pending: output trash coordinates, execute coverage movement, `trash_found(x,y)` via comms.
    - **Ajinkya (World + DWA)**:
        - Done: cafeteria world (walls, tables, bin zone), dynamic obstacle patrol bot.
        - Pending: implement DWA using lidar, output safe (v, ω).
- Reminder: this project is **85% of the module**.
- Todd explains extension to **12th**, due to proposal marking issues.
- Coordinating meetups, avoiding text-file submissions, and emphasising integrated code over zipped standalone projects.

### Progress Status

- Zahin’s global path planner and occupancy map are working; FSM + DWA integration not done.
- Abdul has vision + pattern logic; coverage and comms incomplete.
- Ajinkya’s DWA runs in separate setup but is slow and not integrated with the main project.
- Kunal produces a **report structure draft**; no past examples available.
- Z reinforces repo rules: no text-file code; proper integration required.

### Blockers & Dependencies

- **DWA integration** is path for collector FSM and full navigation loop.
- Zekai’s contributions still unclear; few/no meaningful commits.
- MVP still not ready despite extended deadline.

### Action Items

- [ ]  **Zahin** – Finish collector FSM, integrate A* with DWA once stable, enforce repo standards.
- [ ]  **Abdul** – Complete coverage execution and comms; merge into main once stable.
- [ ]  **Ajinkya** – Deliver working DWA integrated in main project; test with global planner.
- [ ]  **Kunal** – Refine report structure; plan how to present results.
- [ ]  **Zekai** – Push contribution code to GitHub.
- [ ]  **Team** – Meet in person/online to test integrated project and confirm outstanding tasks.

---

### w/c 01/12/2025

### Discussed

- Creation of the **“Real Group”** chat:
    - All project comms must be there; DMs about project will be ignored.
- Decision to **email Todd** about the non-contributing member (Zekai), stating:
    - Zero attendance, zero commits, repeated failed attempts to engage.
    - Request assurance that this won’t penalise active members.
- Realisation the team is **far behind** with ~2 weeks remaining.
- Discovery of a **mystery report** on Canvas not written by the team:
    - Team comments to disown it (“THIS IS NOT OUR REPORT”).
- Discussion of **who focuses on coding vs report**:
    - Mech-background members (Ajinkya, Kunal) lean toward report; Zahin & Abdul more on coding.

### Progress Status

- New world layout: Ajinkya’s office world rearranged into a **grid of tables**; Zahin recalibrates the occupancy map and adds position debugging.
- Abdul’s spotter logic (coverage, zoning, vision) exists; integration with new map and DWA remains fragile.
- Early **supervisor logic** for trash spawn and interactions being considered.
- Kunal contributes comms module (dispatcher + communication); some temporary controllers and test files later removed by Zahin to tidy the repo.
- Overleaf report started, using UoB LaTeX template; abstract/structure drafted, references discussed and tentatively kept.
- Kunal is informally made **project manager**; pastries and donuts used to bribe people into meetings.

### Blockers & Dependencies

- Zekai is formally **removed from the project**.
- DWA still unstable and slow; no final decision yet on its inclusion.
- Zahin and Abdul carry most coding; contributions from others must increase for moderation fairness.
- MVP still missing; results and full report blocked.

### Action Items

- [ ]  **Abdul** – Continue integrating spotter; check and possibly salvage any useful parts of Zekai’s code.
- [ ]  **Ajinkya** – Finalise world layout; confirm DWA status and behaviour.
- [ ]  **Kunal** – Maintain and expand Overleaf report; clean/tick marking criteria in Notion.
- [ ]  **Zahin** – Resplit tasks post-Zekai; centralise comms; integrate A*, mapping, auction, and FSM.
- [ ]  **Team** – Redistribute Zekai’s tasks; push all drafts to repo; confirm in-person meetings and schedule.

---

### w/c 08/12/2025

### Discussed

- Final-week plan and deadlines:
    - Ask for **extension**.
    - Finish **core functionality**, videos, results plots, proofreading, and AI-trace clean up.
- Architectural simplification:
    - Decide whether to **drop DWA** for collectors.
    - Decide how to treat **Spotter** (keep, archive, or abstract via Supervisor).
- Clarify what each person is doing and how they’ll **evidence contribution** (commits, logs, videos, sections).

### Progress Status

- **Supervisor + Auctioneer (Abdul & Zahin)**:
    - Supervisor spawns trash in free spaces, runs auctions, sends tasks to collectors, and despawns trash when collectors arrive.
    - Auction logs show coherent bidding, winners, task completions, and termination when all tasks are done.
- **Collectors (Zahin)**:
    - DWA causes jitter, random paths (under tables), and heavy CPU; collectors moved slowly and misbehaved.
    - Decision: **no DWA for collectors**; replaced by a simpler obstacle avoidance controller.
    - Collectors now: use A* waypoints, respond to supervisor’s auctions, navigate with simple controller, generate logs and ASCII records for experiments.
- **Spotter (Abdul + Kunal)**:
    - Kunal integrates comms + auction into spotter; auctions loop but mis-behave (reauctioning same tasks, weird indices).
    - Combined with DWA issues and timing pressure, team decides:
        - Spotter **de-scoped from main pipeline**; supervisor directly provides trash locations.
        - Spotter’s vision + coverage logic preserved in **archive folder** for report (proof of implementation and planned architecture).
- **Human Agent & DWA (Ajinkya & Kunal)**:
    - Human agent with DWA shows some obstacle avoidance, but still slows simulation and sometimes hides under tables.
    - Treated as **non-core**, nice-to-have for videos only.
- **Testing & Evidence**:
    - Automated testing logs (ASCII, markdown) record auction runs, collector navigation, and trash collection.
    - Team asked to run simulations from different starting conditions to generate evidence.
- **Report (Kunal + team)**:
    - Overleaf report now structured like a 6-page paper:
        - Abstract, intro, methodology, partial results, references.
        - Agreed to keep references (not counted in page limit).
    - To-do: uni logo, individual work descriptions, acknowledgements, final results and figures.

### Blockers & Dependencies

- Official decision: **DWA removed from collectors**; may remain only as an experimental piece for non-core elements.
- Experiments now depend on:
    - Stable **Supervisor → Collectors** pipeline (no spotter in loop).
    - Reliable logs for analysis and result plotting.
- Individual marks depend on:
    - Stated contributions, commit history, report section authorship, and optional meeting attendance.

### Action Items

**Schedule (agreed in chat)**

- [ ]  **Sunday:** Zahin to request extension. Kunal to share updated report draft.
- [ ]  **Tuesday:** Abdul & Zahin to complete **core functionality** (supervisor, auctions, collectors, logging). Ajinkya to start **video recording** of working scenarios.
- [ ]  **Wednesday:** Whole team to fix bugs. Ajinkya to continue/finish videos. Kunal & Ajinkya to plot results & add to report.
- [ ]  **Thursday:** Whole team – Proof-read each other’s sections. Kunal – Remove obvious AI traces from code.

**Per Person (ongoing)**

- [ ]  **Zahin** – Finalise non-DWA collector navigation; keep auction loop correct; maintain testing/logging; write up design decisions and simplifications.
- [ ]  **Abdul** – Finalise supervisor behaviour and any scoring/timing; document coverage + vision logic; help integrate hypothesis setups.
- [ ]  **Kunal** – Lead report completion (structure, references, logos, contributions, results); document comms + auction; help with evaluation chapter.
- [ ]  **Ajinkya** – Produce final videos, support testing, and help with result plots and linking them to system behaviour.