---
title: Known Limitations
sidebar_position: 2
---

# Known Limitations

## Runtime and scale constraints

- Multi-robot high-rate camera streaming can saturate bridge/headset resources.
- Full WebRTC bridge support is currently Jazzy-first in installer workflows.

## Contract-level constraints

- Legacy + profile transport fields must often coexist for compatibility.
- Some modules remain explicit stubs pending broader parity completion.

## Operational caveats

- Dashboard truth depends on app connectivity and workspace state, not publisher presence alone.
- Occupancy map behavior is intentionally workspace-gated in MR runtime.

## Recommendation

Treat SDK changes as integration changes. Validate against:

1. backend runtime,
2. fake data publisher,
3. MR workspace lifecycle,
4. dashboard transition semantics.
