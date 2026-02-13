---
title: Docs Maintenance
sidebar_position: 1
---

# Docs Maintenance

## Branch and deployment

- Canonical docs branch: `docs/site-v1`
- Pages URL: `https://rice-unige.github.io/horus_sdk/`

## Local checks

```bash
npm ci
npm run docs:check
npm run docs:build
```

Serve local build:

```bash
npm run docs:serve
```

## Content rules

- Keep docs aligned with current `main`.
- Prefer runnable commands over abstract descriptions.
- Explicitly mark stub or partial modules.
- Update examples when payload semantics change.

## Review checklist

- broken links: none
- docs quality script: pass
- CI docs workflow: pass
- Pages deploy workflow: pass
