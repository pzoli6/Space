# Artemis II Simulator — Agent Protocol

## Resumption Checklist
When starting a new conversation on this project:
1. Read `docs/PROJECT_STATE.md` — contains validated numbers and the full TODO list
2. Read `.github/copilot-instructions.md` — architecture, conventions, DO NOTs
3. Run `npm run validate` to confirm trajectory integrity before making changes
4. Run `npm run dev` and visually inspect trajectory continuity

## Source of Truth
- Working file: `src/Artemis2Simulator.jsx`
- The file is ~2,640 lines; major section boundaries for searching:
  - `/* ─── FULL MISSION WITH SYNCHRONIZED FLYBY` — trajectory computation start
  - `/* ─── PHASE METADATA & PRESETS` — PH array + PRESETS + VEHICLE_COMPONENTS
  - `/* ═══ HIGH-FIDELITY 3D MODEL BUILDERS` — Three.js mesh builders
  - `/* ═══ MAIN REACT COMPONENT` — App() starts here
  - `/* ─── COMPONENT SVG ILLUSTRATIONS` — ComponentSVG function
  - `function TR` / `function SL` — last ~20 lines

## Known Physics Limitations (by design)
- Phases 1–3 (ascent) use a parameterized kinematic model, not real differential equations
- Phase 10 (outbound coast) uses a smoothstep-bulged lerp, not Keplerian integration
- Phase 14 (return coast) uses a cubic Hermite spline, not Keplerian integration
- The lunar flyby arc (`flybyPointAt`) is scripted, not from Lambert solver
- Ascent → Phase 4 transition has a small positional gap (~20 km) because the ascent model and orbital integration start from slightly different positions

## Validated Numbers to Preserve
Run `npm run validate` before and after changes. These must all pass:
- `closestMoon` between 6,300 and 6,700 km (target: 6,513 km)
- `maxEarth` between 390,000 and 420,000 km (target: 405,500 km)
- All phases 1–19 present in trajectory
- Max position jump < 20,000 km between any two consecutive points

## Constitutional Principles
- SI units always
- No emojis
- Verified data only — mark unknowns as "not officially published"
- Do not break working physics when adding features
- `flybyPointAt` is the single source of truth for flyby geometry

## Priority TODO Queue
Pull next task from this ordered list:
1. **Lambert solver** — replace scripted flyby with real orbital mechanics (see PROJECT_STATE.md §2)
2. **Live news panel** — Anthropic API + web_search on right side (see PROJECT_STATE.md §1)
3. **Astronaut bios** — add Wiseman/Glover/Koch/Hansen to carousel
4. **Engine plumes** — particle systems for SRBs, RS-25s, RL10, AJ10
5. **Post-launch data sync** — search for Artemis II telemetry updates (mission is live)
