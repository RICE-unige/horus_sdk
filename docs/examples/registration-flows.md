---
title: Registration Flows
sidebar_position: 1
---

# Registration Flows

## Single robot flow

```bash
python3 python/examples/sdk_registration_demo.py --robot-name test_bot --with-camera
```

## Multi-robot flow

```bash
python3 python/examples/sdk_registration_demo.py --robot-count 10 --with-camera --with-occupancy-grid
```

## Keep-alive flow for observability

```bash
python3 python/examples/sdk_registration_demo.py --robot-count 4 --with-camera --keep-alive
```

## Payload-level E2E check

```bash
python3 python/examples/e2e_registration_check.py
```

## Notes

- Keep names stable when testing retries/dedupe behavior.
- For stress tests, pair with fake publisher controls (`--vary-camera-resolution`, map options, robot counts).
