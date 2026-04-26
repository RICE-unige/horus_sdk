---
title: Docs Maintenance
sidebar_position: 1
---

# Docs Maintenance

## Branch and deployment

- canonical docs source branch: `main`
- staging branch for large docs refactors: `docs/site-v1`
- published site: `https://rice-unige.github.io/horus_sdk/`

## Local checks

```bash
npm ci
npm run docs:check
npm run docs:build
```

Run the site locally:

```bash
npm run docs:start
```

## Content rules

- keep the root `python/examples/` scripts as the primary documentation path
- mention `python/examples/legacy/` only when it is the paired runtime or validation source
- prefer runnable commands and concrete ROS topic names over abstract descriptions
- keep the Python SDK pages aligned with the actual public methods used in the curated examples
- update the docs whenever registration payload semantics or example ownership changes

## Review checklist

- no broken links
- `npm run docs:check` passes
- `npm run docs:build` passes
- example commands still reflect the current SDK branch
