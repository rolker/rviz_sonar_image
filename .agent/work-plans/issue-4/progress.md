---
issue: 4
---

# Issue #4 — rviz_sonar_image: migrate onto shared marine_colormap

## Implementation
**Status**: complete (pending review)
**When**: 2026-06-07
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Branch**: feature/issue-4

Replaced the local `rviz_sonar_image::ColorMap` (a hand-maintained 13-stop
thermal ramp) with a thin adapter over the shared `marine_colormap`:
- `color_map.{h,cpp}` keep the same class API (`setRange`/`setAlphaRange`/
  `lookup`), so the curtain/fan call sites are unchanged. Internally it holds a
  `marine_colormap::TransferParams` + the canonical `thermal` palette and
  converts `Rgba` -> `Ogre::ColourValue`.
- Behaviour preserved: white-below-floor sentinel, opaque, default -70..0 dB.
  `setAlphaRange` (previously dead code, alpha was hard-coded to 1.0) is now
  wired to the shared alpha ramp; no caller sets it, so rendering is unchanged.
- package.xml + CMakeLists gain the `marine_colormap` dependency.

**Color shift (expected, per ADR-0001):** rviz now uses the canonical de-dup
thermal (the old ramp carried one stop twice and used rounded 0.3 vs 77/255), so
colours shift slightly. The shared lib's golden test locks the reference.

**Validation:** builds clean against the shared lib (only pre-existing
-Wsign-compare warnings). No unit tests in this repo (rviz plugin); the color
math is now covered by marine_colormap's tests. Runtime/visual check in rviz
deferred (needs a display) — candidate for /verify.

**Dependency note:** this surfaced that marine_colormap was building STATIC
(no -fPIC) -> can't link into the rviz plugin .so. Fixed upstream
(rolker/marine_colormap#4, build SHARED) and merged before this.

Closes #4. Part of rolker/unh_marine_autonomy#137.
