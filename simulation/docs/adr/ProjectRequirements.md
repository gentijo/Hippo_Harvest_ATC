# Robotics Software Engineer Project

## Overview

This project is intended to provide insight into the way you think about robotic systems and trade-offs. The subject is inspired by a real problem that we need to solve in our farm. The goal is not to determine the “right” answer, but to show us your thought process, and then display your ability to build prototype code that demonstrates your treatment of some of the nuances of your approach.

## The problem

We have a number of robots operating in the farm - on the order of 20 - that are moving around the space 24/7, often with little or no human supervision. These robots operate in close proximity to each other as they roam through the farm. Each robot is equipped with a planar laser with a 140 degree field of view, facing forward, with 70 degrees on each side of the front-facing midline of the robot. At top speed, robots move 2 m/s.

We need to build an air traffic control system for our robots. This system should be sufficient to ensure that robots do not hit each other, and can reach their destinations without deadlocks. As you design the system, a few things to keep in mind:

- You can assume perfect localization and a known map. You can supplement the map with any additional data structures that would help you generate the solution, and assume that all robots and any centralized components all have access to this map and supplemental information.
- You can also assume that robots have constant connectivity with each other and with any centralized components you may build.
- You need not consider collision avoidance with humans or other physical infrastructure.
- We need a solution that will allow robots to back up safely, moving for short distances in directions where they have no lidar coverage.

## Deliverables

### A write-up of your approach

Please share a document where you describe your approach, and articulate why you have chosen this strategy, as well as any limitations, caveats, or risks associated with the approach.

### A prototype illustrating some aspect of the strategy

You do not need to implement a full and comprehensive system - that’s definitely overkill. Instead, we would like for you to code up some interesting aspects of your approach that can be demonstrated in isolation. We encourage you to use whatever form of AI assistance you deem appropriate, but be prepared to discuss your code and the design decisions you made.

Please make a private repository on github with your code and share it with user egiljoneshippo, and include instructions on how to run it.

## Post-submission phone screen

After your submission, we’ll schedule a 45 minute follow-on phone screen to discuss your approach, walk through the code, and think through some potential extensions.
