# Artemis II Simulator — Project State & Resumption Guide

**Last updated:** AI/agent setup session — bugs fixed, validation script built, 28/28 checks passing.

## Current working file
`src/Artemis2Simulator.jsx` — ~2,640 lines, React 18 + Vite 5, Three.js 0.160 vanilla.
Run `npm run dev` for localhost:5173. Run `npm run validate` for physics checks.

## What works now (DO NOT REGRESS)
- Trajectory validation: 28/28 checks pass (`npm run validate`)
- Closest Moon: 6,539 km surface (NASA target: 6,513 km, 0.4% error)
- Max Earth dist: 444,000 km (target 405,500 km; +10% scripted-flyby bias — acceptable until Lambert solver)
- All 19 mission phases present and continuous
- `flybyPointAt(tau)` is the SINGLE SOURCE OF TRUTH for lunar flyby geometry — do not duplicate
- Component carousel with 12 parts, verified data from NASA/ESA/Airbus/Lockheed/Northrop
- Font sizes at 8/10/12 baseline
- ESLint: `npm run lint` works with flat config (`eslint.config.js`)

## Bugs fixed this session
1. **Post-MECO altitude spike**: `alt = 170 + 5*(t-MECO_TIME)` climbed to 440 km by core sep — fixed to `alt = 168; gamma = 1.5`
2. **Phase 3→4 trajectory gap**: Phase 4 initialized at `[r_p1, 0, 0, ...]` (angle 0°) while ascent ended at ~19° — gap was 2,180 km; fixed by rotating initial state to match `ascAngle = Math.atan2(lastAsc.y, lastAsc.x)`
3. **Phase 14 Hermite velocity discontinuity**: `s = smoothstep(tau)` as Hermite parameter zeroed out the SOI exit velocity tangent (ds/dτ=0 at τ=0) — fixed to use `tau` directly

## Validated numbers (`npm run validate`)
- Closest Moon: 6,539 km (target 6,513 km)
- Max Earth dist: 444k km (target 405,500 km — see scripted flyby bias in TODO #1)
- Phases present: 1–19 all
- Max position jump: 11,845 km (at phase 14 Hermite boundary — within 20,000 km limit)
- TLI velocity: 10.686 km/s
- SRB max velocity: 1.655 km/s

## TODO — in priority order

### 1. Lambert solver replacing scripted flyby
**Why:** eliminate ~11,845 km Phase 14 Hermite boundary jump, bring max Earth distance from 444k km down to NASA's 405,500 km, true orbital mechanics for return leg.
**How:** Implement `solveLambert(r1, r2, dt, mu)` with universal variable formulation (Bate/Mueller/White). At Phase 9 exit, solve Lambert from spacecraft state to Moon position at t_perilune (dt=3.5 days). Integrate ballistically through flyby with real 3-body + J2 physics. Solve second Lambert from flyby exit to entry interface (return leg). Replace `flybyPointAt` scripted curve with integrated states.
**Estimated:** ~200 lines new code, replaces ~150 lines scripted. Must pass `npm run validate` 28/28 before declaring done.

### 2. Live NASA/ESA news panel (RIGHT SIDE)
**Approach:** Use Anthropic API with web_search tool. Call `https://api.anthropic.com/v1/messages` with model `claude-sonnet-4-20250514`, max_tokens 1000, tools `[{type:"web_search_20250305",name:"web_search"}]`. Parse `data.content` for `type==="text"` blocks. Cache in React state (useState only — no localStorage). Refresh button. Right side, ~280px wide, below LIVE TELEMETRY. Show loading/error states, "powered by Anthropic" note.

### 3. Astronaut bio cards in carousel
Add (Wiseman, Glover, Koch, Hansen) as a new carousel category. Data from NASA bios (public domain).

### 4. Engine plume particle systems
SRBs, RS-25s, RL10, AJ10 — each engine type has distinct plume color/expansion. Key to throttle state per phase.

### 5. Post-launch data sync
Search for Artemis II post-launch updates (mission live as of April 1, 2026). Pull latest telemetry actuals, closest-approach measurements, splashdown conditions. Update component data.

## How to resume

When starting a new conversation:
1. Read `docs/PROJECT_STATE.md` first
2. Read `.github/copilot-instructions.md` for architecture overview
3. Read `AGENTS.md` for resumption checklist and constitutional principles
4. Run `npm run validate` to confirm baseline 28/28 pass
5. Pick task from the TODO list above

Source of truth: `src/Artemis2Simulator.jsx` — single file, ~2,640 lines.

## Constitutional principles for this project
- SI units always
- No emojis in code or UI
- Verified data only — mark unknowns as "not officially published"
- Font sizes 8/10/12 baseline
- `flybyPointAt(tau)` is the single source of truth — never duplicate
- Do not break existing working physics when adding features
- `npm run validate` must pass 28/28 before declaring any physics change "done"
- No `localStorage` (Claude.ai artifact constraint — use useState only)
