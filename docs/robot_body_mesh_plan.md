# Robot Body Mesh Plan (Horus SDK)

Status: Planning only. No implementation in this PR.

## Goal
Provide SDK-side contracts for robot body mesh selection and future mesh delivery, aligned with the Horus MR robot body mesh plan.

## Phase 1 (next immediate implementation): known robots only
Send model identity metadata during registration so Horus MR can spawn local prepackaged prefabs/meshes.

### Strategy
1. Add registration metadata field (planned): robot_model_id.
2. Populate robot_model_id in demos/examples for known robots.
3. Keep robot description collision/joint payload unchanged as fallback path.
4. Do not transfer full mesh assets from SDK in this phase.

### Constraints
- Registration changes remain backward-compatible.
- workspace_scale is still managed by MR; SDK only sends identity/metadata.
- No large binary mesh transfer over ROS topics for known robots.

## Phase 2 (future): unknown robots
Add optional manifest + cached fetch flow for robots not bundled in MR app.

High-level direction:
- Registration includes mesh manifest (mesh ids, hashes, link map, optional source uri).
- MR fetches only cache misses and stores by hash.
- Runtime traffic stays TF/state-only after warmup.
- ROS remains metadata/control channel; heavy binary transfer is not the default path.

## Candidate known robot model IDs (first batch)
- unitree_go1
- unitree_go2
- bostondynamics_spot
- anymal
- clearpath_jackal
- clearpath_husky
- turtlebot3_burger
- turtlebot3_waffle
- rosbot_2r
- rosbot_xl

## Non-goals for immediate phase
- Generic remote mesh ingestion for arbitrary URDF packages.
- Continuous mesh streaming over ROS every session.
- Auto-conversion pipeline for all vendor mesh formats.

## Acceptance criteria for Phase 1
- Known robot registrations include stable robot_model_id.
- Existing SDK demos continue to run without regressions.
- If robot_model_id is absent, current robot description fallback remains intact.
- Host/joiner sessions resolve consistent model identity for the same robot.
