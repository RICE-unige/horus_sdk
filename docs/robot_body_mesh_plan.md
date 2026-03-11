# Robot Body Mesh Plan (Horus SDK)

Status: Implemented V1 path for Nova Carter through robot-description visual mesh transport. More tuning remains.

## Current direction
The SDK no longer treats shipped MR-side prefabs as the primary robot body strategy.

Current working path:
1. Resolve URDF/xacro and visual meshes on the SDK side.
2. Normalize supported mesh sources into a baked internal geometry representation.
3. Send robot-description payloads with:
   - collision data
   - joint data
   - visual mesh groups
   - baked mesh assets
4. Let Horus MR reconstruct the robot body from that payload.

This is now the primary body path for Nova Carter.

## What is implemented now
- Robot-description payloads can carry visual mesh groups and baked mesh assets.
- Visual groups are no longer compiled as one monolithic shell only.
- Fixed-joint chains are collapsed together.
- Non-fixed child links remain separate visual groups so MR can bind them to TF frames directly.
- Large visual-mesh description payloads use paced chunk transport instead of a raw burst.
- Carter live demo prepares the robot-description mesh payload before registration and bridge/app registration now supports pre-seeding.

## Source mesh format support today
These are SDK-side source formats. The SDK normalizes them before Horus MR sees anything.

### Supported directly
- `.obj`
  - Parsed directly.
  - Basic `MTL` diffuse color hints are preserved when available.
- `.stl`
  - Parsed directly.

### Supported through SDK-side Blender normalization
- `.dae`
  - Converted through Blender CLI on the SDK machine.
- Other mesh formats Blender can reliably import and convert
  - Potentially usable through the same path.
  - Not treated as officially supported unless explicitly validated.

### Not first-class supported today
- `.fbx`
  - Feasible through Blender normalization.
  - Not currently declared as a tested supported path.
- `.usd` / `.usda` / `.usdc`
  - Same status: possible in principle, not currently supported in practice.
- Full textured/material-rich authoring assets
  - Out of scope for the current payload format.

## What the SDK sends to MR
The SDK does not send original vendor mesh files to Horus MR.

It sends normalized baked mesh payload data:
- vertex positions
- normals
- triangle indices
- bounds
- simple color hints

That means Horus MR does not need runtime support for OBJ/FBX/DAE/USD as file formats.

## Current limits
- Color support is basic and geometry-first.
- Full material graphs and textures are not transported.
- Format support is normalization-based, not universally format-agnostic.
- Only formats that the SDK parser or Blender normalization path can handle correctly are usable.

## Why local bundled models were deprioritized
Bundled local models are still valid as a future optimization tier, but they are no longer required for the core robot body workflow.

They remain useful later for:
- hand-optimized Quest assets
- curated materials/textures
- lower runtime payload size for common robots
- offline/local fallback

But they are no longer necessary for functional robot body delivery.

## Future work
### 1. Appearance quality
- Preserve more per-part visual color/material hints.
- Evaluate whether selective texture support is worth the complexity.
- Improve body material defaults without falling back to collision-style rendering.

### 2. Format coverage
- Validate and promote `.fbx` if Blender normalization proves clean and repeatable.
- Evaluate practical `USD` support only if real robot packages require it.
- Add explicit tests per supported source format.

### 3. Transport hardening
- Tune chunk sizes and pacing for larger robot bodies.
- Add better diagnostics around incomplete description delivery.
- Consider optional compression refinements for large assets.

### 4. Optional bundled supported-robot tier
- Reintroduce stable `robot_model_id`-based local models only as an optimization/polish tier.
- Keep robot-description mesh transport as the generic/default path.

## Non-goals right now
- Universal support for every robot mesh format.
- Runtime import of vendor authoring files in MR.
- Full texture/material graph transport.
- Requiring a bundled local prefab for robot body support.

## Acceptance criteria for the current path
- SDK resolves robot visual meshes from robot description sources.
- SDK emits TF-bindable visual groups plus baked mesh assets.
- Carter live demo registers immediately after mesh prep and bridge readiness.
- Host/joiner both receive the same robot body through the existing description path.
- Fallback collision/joint behavior remains available when visual mesh data is missing.
