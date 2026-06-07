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

## Integrated Review
**Status**: complete
**When**: 2026-06-07 16:20 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #5 at `e5c8beb`
**Sources**: 1 at head (Copilot R1 @ `e5c8beb`) + prior timeline (Implementation entry; no prior review)
**Cross-source confirmations**: 0
**CI**: copilot-pull-request-reviewer success (no build/test check on repo; built clean locally)

### Findings
- [ ] (suggestion/defensive, Copilot R1) lookup() raw-derefs palette_ — can't be null today (set once from guaranteed built-in "thermal", never reassigned; no set_type), but a one-line guard/fallback is cheap insurance for a per-frame render-path pointer — `src/color_map.cpp:36`

### False positives
- (Copilot R1) `TransferParams params_` "may leave fields indeterminate (POD)" — `src/.../color_map.h:29`: marine_colormap::TransferParams declares in-class default member initializers for every field (min{0}, max{1}, gain{1}, contrast{1}, alpha_ramp{false}, ...), so a default-constructed params_ is fully defined. Not a POD-without-initializers; no UB.
