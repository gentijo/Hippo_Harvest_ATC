# ADR 0003: Use a Centralized Traffic Manager for Local Conflict Resolution

- Status: Proposed
- Date: 2026-06-09

## Context

The project brief describes an air traffic control problem for roughly 20 robots operating continuously in close proximity with little or no human supervision. The system must help robots avoid collisions and reach their destinations without deadlocks.

The brief also permits several simplifying assumptions that make a centralized coordination prototype practical:

- perfect localization
- a known shared map
- optional supplemental map data structures
- constant connectivity between robots and centralized services

The current simulation already exposes robot pose, navigation state, and map information through ROS 2 topics. That creates an opportunity to prototype a centralized coordination module that supervises local robot conflicts without needing to solve full fleet scheduling.

## Decision

We will prototype an `air_traffic_control` or `traffic_manager` ROS 2 module that monitors robot motion and intervenes when two robots are converging within an unsafe distance.

This prototype will use perfect localization, shared map knowledge, and centralized robot-state awareness as its primary source of traffic information rather than lidar-driven local conflict detection.

For each robot, the module will observe:

- current pose
- current navigation goal
- nearest-neighbor distance
- whether nearest-neighbor distance is increasing or decreasing

When the nearest-neighbor distance is decreasing and falls below a minimum safety threshold, the module will:

1. determine which robot should yield
2. pause that robot’s current navigation goal
3. record the paused goal for later resumption
4. inject a temporary occupied circle around the paused robot so other robots plan around it
5. allow the higher-priority robot to continue

When the conflict condition clears, the module will:

1. remove the temporary occupied region
2. restore the paused robot’s goal
3. allow the paused robot to proceed

## Rationale

This design fits the project brief because it demonstrates system-level reasoning and coordination trade-offs without requiring a full production traffic management stack.

It also takes advantage of the assumptions in the problem statement:

- Perfect localization makes nearest-neighbor distance and conflict detection reliable enough for a prototype.
- A known map allows the system to place temporary keep-out regions around paused robots.
- Constant connectivity makes centralized arbitration feasible.

The design is intentionally local and reactive. It does not attempt to globally optimize all robot motion. Instead, it focuses on resolving immediate collision risks in a way that is explainable, easy to prototype, and easy to demonstrate visually.

The prototype intentionally does not use lidar as the primary mechanism for traffic conflict detection.

### Why not use lidar as the primary signal

- The project brief explicitly allows perfect localization and a shared known map, which provide cleaner and more direct inputs for centralized traffic reasoning.
- Lidar is more valuable for local obstacle perception than for proving the coordination logic of a centralized traffic-control prototype.
- Raw lidar observations require additional interpretation, including object association, motion inference, filtering, and disambiguation of what belongs to another robot versus static structure.
- A localization-driven approach makes nearest-neighbor distance, closing rate, and goal progress explicit and easier to reason about than inferred proximity from range scans.
- Using localization and shared state better matches the air-traffic-control framing, where the coordinator reasons over fleet positions and intent rather than only local sensing.
- The brief only requires safe backing in regions without lidar coverage, which further supports not centering the design on lidar availability.
- For a prototype, lidar-heavy logic would add implementation complexity in areas that are not central to the design question being evaluated.

### Additional advantages of relying on perfect localization and shared state

- conflict decisions are deterministic and easier to reproduce
- behavior is easier to explain in a design review
- debugging is easier because robot positions and conflict geometry are directly observable
- temporary occupied zones can be generated from exact robot pose rather than estimated obstacle contours
- the prototype stays focused on coordination policy instead of perception uncertainty

### Trade-offs of this choice

- The prototype will not demonstrate robustness to perception noise, occlusion, or missed detections.
- It assumes the localization and shared-state infrastructure is trustworthy.
- A production system may still need lidar or other local sensing as a safety layer even if centralized coordination uses map and pose data.

## Conflict Resolution Model

The traffic manager will operate as a local conflict supervisor:

- monitor robot state continuously
- identify the nearest neighbor for each robot
- detect when separation is shrinking
- intervene only when both proximity and closing behavior indicate risk

A deterministic yielding policy is required to avoid both robots stopping at once. A simple prototype policy is:

- the robot with the shorter remaining distance to its goal proceeds
- the robot with the longer remaining distance yields
- ties are broken deterministically, for example by robot ID

This keeps the system simple while reducing ambiguity during conflict resolution.

## Use Cases

### 1. Head-on aisle conflict

Two robots move toward each other in the same corridor. Their separation decreases below the safety threshold.

Expected behavior:

- the robot farther from its goal yields
- its goal is paused and stored
- a temporary occupied circle is placed around it
- the other robot proceeds through the corridor
- once the path clears, the paused robot resumes

This reduces the chance of both robots entering a deadlock in a narrow passage.

### 2. Intersection merge conflict

Two robots approach the same crossing from different directions.

Expected behavior:

- the traffic manager detects converging motion into the same local space
- one robot is selected to proceed based on the priority rule
- the other robot pauses before entering the conflict region
- the paused robot resumes after the crossing clears

This prevents simultaneous entry into the same intersection.

### 3. Rear-approach or overtaking conflict

A trailing robot closes on a slower or stopped robot ahead.

Expected behavior:

- the shrinking separation triggers conflict evaluation
- the lead or blocked robot is represented as a protected occupied zone
- the trailing robot either replans around it or yields if no safe bypass exists

This helps avoid low-speed collisions caused by queueing or stalled motion.

### 4. Robot stopped near a table or workstation

A robot becomes stationary in a location that partially blocks a lane near infrastructure.

Expected behavior:

- the paused or stuck robot is treated as a temporary obstacle
- approaching robots are routed around it if free space exists
- if no safe bypass exists, approaching robots pause before reaching unsafe range

This makes blocked robots visible to the planner as traffic constraints rather than only as dynamic motion failures.

### 5. Congestion near home or dispatch area

Multiple robots start close together and immediately compress spacing.

Expected behavior:

- the traffic manager serializes movement through local yielding
- lower-priority robots are paused briefly
- the first robot exits the shared area
- paused robots are resumed in turn

This reduces startup bunching and launch-area deadlocks.

### 6. Short backing maneuver with limited lidar coverage

A robot needs to reverse a short distance into an area not fully covered by its forward-facing lidar.

Expected behavior:

- the traffic manager protects the backing robot’s local space
- nearby robots yield before entering the reverse path
- the backing maneuver completes with a temporary protected zone

This is especially relevant because the project brief explicitly requires safe backing behavior.

### 7. Conflict in visually ambiguous sensing conditions

Two robots are close to tables, shelves, or other map structure where lidar returns might be difficult to interpret cleanly.

Expected behavior:

- the traffic manager still reasons from known robot poses and shared state
- conflict detection does not depend on interpreting noisy or ambiguous range returns
- yielding and resume behavior remain deterministic

This supports the argument that centralized traffic control should be driven by fleet state and intent rather than by local sensing interpretation in this prototype.

## Deadlock Minimization Strategy

This approach is designed to minimize deadlocks by using asymmetric, deterministic yielding.

Key anti-deadlock measures:

- only one robot yields in a given pairwise conflict
- priority is determined deterministically
- paused goals are restored automatically after clearance
- conflicts are handled locally and temporarily rather than freezing the whole fleet

The prototype should also avoid intervening on proximity alone. Distance should be combined with directional evidence such as a decreasing separation trend so robots that are already moving apart are not paused unnecessarily.

## Risks

### Oscillation

If pause and resume thresholds are too similar, robots may repeatedly stop and restart.

Mitigation:

- use hysteresis
- define a tighter threshold for pausing and a larger clearance threshold for resuming

### False conflict detection

Two robots may be close but not actually on conflicting paths.

Mitigation:

- combine distance with closing rate
- optionally incorporate heading or path-overlap checks

### Overconfidence in localization

If localization or shared robot state is wrong, the traffic manager may make poor conflict decisions because it is not independently validating robot positions through lidar.

Mitigation:

- treat this as an accepted prototype assumption from the project brief
- keep the limitation explicit in documentation
- reserve lidar or other local sensing as a future safety-layer extension rather than part of the primary prototype logic

### Deadlock chains

Pausing one robot may indirectly block additional robots behind it.

Mitigation:

- monitor downstream congestion
- prefer yielding before entering chokepoints when possible

### Overblocking due to temporary occupied circles

The protected zone around a paused robot may remove all feasible paths in narrow areas.

Mitigation:

- tune the occupied radius carefully
- place yielding robots in safer pull-off positions when possible

### Starvation

The same robot may repeatedly be selected to yield.

Mitigation:

- add fairness rules
- track repeated yields and elevate priority for delayed robots

### Resume instability

When the stored goal is restored, the robot may immediately re-enter the same conflict.

Mitigation:

- reevaluate the environment before resuming
- delay resume until the conflict geometry is genuinely cleared

### Limited scalability of pairwise logic

Nearest-neighbor reasoning may not capture complex three-robot or four-robot interactions.

Mitigation:

- treat this as a prototype limitation
- extend later toward conflict zones, reservation systems, or richer multi-agent coordination if needed

## Consequences

### Positive

- simple, explainable coordination policy
- good fit for the project’s prototype-oriented scope
- leverages the project’s shared-map and perfect-localization assumptions
- avoids spending prototype complexity on lidar interpretation and perception plumbing
- easy to visualize in RViz using markers and temporary occupied regions
- directly demonstrates treatment of collision and deadlock trade-offs

### Negative

- reactive rather than globally optimal
- may struggle in dense traffic or multi-robot chain conflicts
- depends on careful threshold tuning
- may introduce stop-and-go behavior if not damped with hysteresis and fairness logic
- does not evaluate perception uncertainty or lidar-driven ambiguity

## Alternatives Considered

### Fully decentralized avoidance only

Rejected for the prototype because it is harder to explain, harder to demonstrate centrally, and less aligned with the air-traffic-control framing of the project.

### Lidar-first local conflict detection

Rejected for the prototype because it adds interpretation complexity without providing the same direct value as the project’s allowed perfect-localization and shared-map assumptions. It shifts attention from coordination policy to perception processing, which is not the main point of the prototype.

### Full global reservation or time-slot planner

Rejected for the prototype because it adds substantial complexity beyond what is needed to demonstrate the core idea.

### Pure log-based monitoring without intervention

Rejected because it observes conflicts but does not actively coordinate robots to avoid them.

## Follow-up

- Define the ROS 2 interfaces for robot pose, goal, pause, and resume behavior.
- Define the minimum safety threshold and clearance threshold.
- Add RViz visualization for protected occupied circles and paused robots.
- Log every pause, resume, and arbitration decision for traceability.
- Evaluate whether pairwise nearest-neighbor logic is sufficient before extending to richer conflict-zone management.
