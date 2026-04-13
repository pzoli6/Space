# Artemis II Simulator — GitHub Copilot Instructions

## Project Overview
High-fidelity React + Three.js simulator of the Artemis II crewed lunar mission (launched April 1, 2026). Simulates all 19 mission phases from launch to Pacific splashdown with physically-grounded trajectory mathematics.

## Tech Stack
- React 18 + Vite 5
- Three.js 0.160 (direct, no react-three-fiber)
- Pure JS physics (no external math library)
- Single component file: `src/Artemis2Simulator.jsx` (~2,640 lines)

## Code Style & Conventions
- SI units everywhere: km, km/s, seconds, km³/s²
- No emojis in code or UI
- Font sizes at 8/10/12 baseline (UI)
- Verified data only — mark unknowns as "not officially published"
- Physics constants at top of file (MU_E, MU_M, R_E, R_M, etc.)

## Key Physics Architecture
- `computeFullMission(params)` returns `{ pts, minMD, minMI, maxED }` where `pts` is the full trajectory array
- `eom3D(th0, useJ2)` returns equations of motion closure for Earth-Moon 3-body + J2 perturbation
- `rk4(state, t, dt, f)` — 4th-order Runge-Kutta integrator; `state = [x,y,z,vx,vy,vz]` in km/km/s
- `moonPos3D(t, th0)` / `moonVel3D(t, th0)` — Moon ephemeris at time t
- `flybyPointAt(tau)` — SINGLE SOURCE OF TRUTH for lunar flyby arc [tau ∈ 0,1]; do NOT duplicate this geometry
- Coordinate system: physics XY = orbital plane, Z = inclination axis; Three.js maps: x→x, y→z_3d, z→y_3d

## Phase Map (do not renumber)
| Phase | ID | Name |
|-------|----|------|
| 0 | PRE-LAUNCH | (unused in trajectory) |
| 1 | SRB BURN | 0–128 s |
| 2 | CORE STAGE | 128–486 s |
| 3 | CORE SEPARATION | 486–540 s |
| 4 | ORION DEPLOY | 540–2940 s |
| 5 | ICPS PERIGEE RAISE | ~2940 s |
| 6 | HEO COAST | ~2940–6480 s |
| 7 | APOGEE RAISE | ~6480 s |
| 8 | HIGH EARTH ORBIT | ~6480–92237 s |
| 9 | TLI BURN | ~92237 s + 349 s |
| 10 | OUTBOUND COAST | TLI exit → lunar SOI |
| 11 | LUNAR SOI ENTRY | flyby tau 0–0.4 |
| 12 | LUNAR FLYBY | flyby tau 0.4–0.6 |
| 13 | LUNAR SOI EXIT | flyby tau 0.6–1.0 |
| 14 | RETURN COAST | SOI exit → entry interface |
| 15 | PEAK HEATING | entry 0–30% |
| 16 | HYPERSONIC ENTRY | entry 30–65% |
| 17 | DROGUE CHUTES | entry 65–80% |
| 18 | MAIN CHUTES | entry 80–98% |
| 19 | SPLASHDOWN | entry 98–100% |

## Validated Trajectory Numbers (run `npm run validate` to verify)
- Closest Moon surface: ~6,513 km (NASA: 6,513 km, 0.0% error target)
- Max Earth distance: ~405,000 km (NASA: 405,500 km)
- All 19 phases must be present
- Max position jump between consecutive points: < 20,000 km

## Component Structure
- `App()` — main React component, manages all state, Three.js lifecycle, UI
- `buildSLS()` / `buildOrion()` / `buildCMEntry()` / `buildCMChutes()` — Three.js group builders
- `ComponentSVG({ id })` — SVG technical illustrations for carousel
- `TR({ l, v, c, big })` — telemetry row display helper
- `SL({ label, unit, min, max, step, val, set, fmt })` — slider control helper

## DO NOT
- Add `react-three-fiber` or `@react-three/drei` — project uses vanilla Three.js  
- Duplicate the `flybyPointAt` geometry anywhere else
- Change the physics constants (MU_E, MU_M, R_M, etc.) without verification
- Use `localStorage` (targets Claude.ai artifact environment which restricts it)
- Break existing working phases when adding features
- Add emojis to UI

## Candidate TODOs (from PROJECT_STATE.md)
1. Lambert solver replacing scripted flyby (eliminate residual position jumps)
2. Live NASA/ESA news panel using Anthropic API with web_search tool
3. Astronaut bio cards in carousel (Wiseman, Glover, Koch, Hansen)
4. Engine plume particle systems for each engine type
5. Earth rotation synced to real sidereal time (already started: `earth.rotation.y = p.t * 7.2921e-5`)

## Testing
```
npm run validate   # trajectory sanity checks
npm run lint       # ESLint
npm run dev        # dev server at localhost:5173
```
