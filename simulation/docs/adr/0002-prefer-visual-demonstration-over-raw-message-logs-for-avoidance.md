# ADR 0002: Prefer Visual Demonstration Over Raw Message Logs for Robot Avoidance Evaluation

- Status: Accepted
- Date: 2026-06-09

## Context

This project needs a clear way to demonstrate that multi-robot motion, spacing, and avoidance behavior are working correctly in simulation.

The project brief describes an air traffic control problem for roughly 20 robots operating continuously in close proximity. The system must help robots avoid collisions and reach destinations without deadlocks, while the prototype only needs to illustrate important aspects of the strategy in isolation.

One possible approach is minimal-effort evidence collection through terminal or file logging, such as:

- pose streams
- `cmd_vel` streams
- odometry updates
- waypoint arrival messages

Another approach is to present the same robot behavior visually in a simulation view, where an observer can see:

- where robots are in relation to each other
- when one robot yields or waits
- whether a robot is blocked by another robot or a table
- whether route planning and recovery behavior make sense spatially
- whether the system behavior matches what the logs claim happened

The project assumptions of perfect localization, a shared known map, and continuous connectivity make spatial interpretation especially valuable, because the main questions are about coordination logic, deadlock avoidance, and safe motion policy rather than uncertainty in perception.

For this project, the goal is not only to record motion data, but to make avoidance behavior understandable, debuggable, and credible to engineers and reviewers.

## Decision

We will prefer visual presentation as the primary method for demonstrating robot avoidance behavior.

Raw logs of pose, `cmd_vel`, and related topics may still be collected as supporting evidence, but they are not the main artifact used to explain or validate avoidance performance.

## Rationale

### Why visual presentation is better

- Avoidance is fundamentally spatial and temporal. A visual view shows both at once.
- Reviewers can quickly see whether robots are genuinely avoiding one another or merely stopping for unrelated reasons.
- Visual playback makes it easier to distinguish between planner issues, control lag, localization artifacts, and physical blockage.
- It reduces ambiguity. A stream of `cmd_vel` messages may show commands changing, but it does not clearly show whether the robot actually avoided another robot in the environment.
- It is faster to inspect. A human can recognize stuck robots, near misses, bunching, and route conflicts almost immediately in RViz.

### Why raw logs are not enough by themselves

- Pose and `cmd_vel` logs are difficult to interpret without spatial context.
- Logs do not naturally convey relative positions between multiple robots and obstacles.
- Large message dumps create noise and make it harder to identify the important event.
- Logs can show that commands were issued, but not whether the resulting visible behavior was acceptable.
- For multi-robot systems, log-only evidence scales poorly as robot count increases.

### Why this matters specifically for this project

- The project emphasizes multi-robot behavior in a shared workspace.
- The stated problem is specifically about collision prevention and deadlock avoidance for a fleet-scale operating pattern.
- The team is actively debugging issues such as robots getting stuck, falling behind, colliding with tables, or appearing visually out of sync.
- These issues are much easier to diagnose in a visual presentation than in raw terminal output.
- A visual artifact is more persuasive for design reviews, demos, and project evaluation than a large log file.
- The deliverable asks for a write-up plus a prototype that demonstrates important aspects of the approach, and visual evidence supports that requirement much better than message dumps alone.

## Consequences

### Positive

- Faster debugging of robot avoidance and routing behavior
- Clearer communication during demos and design reviews
- Better ability to spot emergent multi-robot issues
- Lower cognitive load than reviewing large volumes of message logs

### Negative

- Visual validation alone is not enough for quantitative analysis.
- Some subtle timing or state-transition details may still require logs for root-cause analysis.
- Producing a good visual presentation requires maintaining useful RViz displays, markers, and robot overlays.

## Alternatives Considered

### Log-first evidence only

Rejected as the primary method because it is low-effort to produce but high-effort to interpret, especially for multi-robot avoidance behavior.

### Visual-only evidence with no logging

Rejected because logs still provide useful supporting detail when diagnosing controller, planner, or orchestration failures.

### Combined approach with visual presentation as primary and logs as secondary

Accepted because it balances quick human understanding with engineering traceability.

## Follow-up

- Keep visual simulation outputs, markers, paths, and robot overlays as first-class demo artifacts.
- Continue publishing logs for debugging and post-run investigation.
- Use logs to support the visual story rather than replace it.
