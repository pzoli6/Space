# Artemis II Simulator — Project State & Resumption Guide

**Last updated:** Session ending at low-token state, trajectory sync + carousel complete.

## Current working file
`/mnt/user-data/outputs/artemis2-3d-simulator.jsx` — 2,416 lines, 118 KB, validated working.

## What works now (DO NOT REGRESS)
- Trajectory sync: 6,539 km closest Moon approach (NASA: 6,513 km, 0.4% agreement)
- All 19 mission phases continuous, max boundary jump 1,320 km
- `flybyPointAt(tau)` helper is the single source of truth — do not duplicate flyby geometry anywhere else
- Component carousel with 12 parts, verified data from NASA/ESA/Airbus/Lockheed/Northrop
- Font sizes at 8/10/12 baseline
- File structure: part1 (physics + VEHICLE_COMPONENTS) → part2 (3D models) → part3 (React UI + ComponentSVG)
- Source parts in `/home/claude/artemis_part{1,2,3}.txt` — rebuild with `cat part1 part2 part3 > outputs/artemis2-3d-simulator.jsx`

## Validated numbers (from `/home/claude/validate_sync.js`)
- Closest Moon: 6,539 km surface
- Max Earth: 450,499 km
- Phases present: 1–19 all
- Max position jump: 5,716 km (inside phase 10 ballistic motion, not a teleport)

## TODO — in priority order

### 1. Live NASA/ESA news panel (RIGHT SIDE)
**Approach:** Use Anthropic API in artifacts with web_search tool. Call `https://api.anthropic.com/v1/messages` with model `claude-sonnet-4-20250514`, max_tokens 1000, and `tools: [{type: "web_search_20250305", name: "web_search"}]`. Prompt: "Find 5 most recent Artemis II news articles from nasa.gov, esa.int, airbus.com. Return as JSON array with fields: title, source, date, url, summary (20 words max)." Parse `data.content` filtering for `type === "mcp_tool_result"` and `type === "text"` blocks. Cache in React state. Refresh button. Place on right side, width ~280px, below LIVE TELEMETRY panel. Show loading spinner, error fallback, "powered by Anthropic" note.

**Must not use localStorage** (Claude.ai artifacts restriction). Use useState only.

### 2. Replace scripted flyby with Lambert solver
**Why:** eliminate 1,320 km boundary jump, get true orbital mechanics for return leg, improve max-Earth distance from 450k to NASA's 405k.
**How:** Implement `solveLambert(r1, r2, dt, mu)` returning v1, v2. Use universal variable formulation (Bate/Mueller/White). At phase 9 end, solve Lambert from spacecraft state to Moon position at t_perilune with dt = 3.5 days. Integrate ballistically through the flyby using real 3-body physics. Solve second Lambert from flyby exit to entry interface position with dt = return leg duration. Replace `flybyPointAt` scripted curve with integrated state.
**Estimated:** ~200 lines new code, replaces ~150 lines scripted. Test against `validate_sync.js`.

### 3. Carousel enhancements
- Add astronaut bio cards (Wiseman, Glover, Koch, Hansen) as new category
- Add "Mission Timeline" category with hour-by-hour events from NASA press kit
- Replace hand-drawn SVGs with fetched NASA imagery where copyright permits (NASA imagery is public domain)
- Add interactive hotspots on main SVGs that show subcomponent detail on hover

### 4. Animation improvements
- Engine plume particle systems for SRBs, RS-25s, RL10, AJ10
- Earth rotation synced to real sidereal time
- Moon libration (small wobble)
- Stage separation animations with physics (SRBs tumbling, core stage drifting)
- Heat shield glow intensity keyed to atmospheric density profile

### 5. Re-verify against latest NASA data
- Search for Artemis II post-launch updates (mission is live as of April 1, 2026)
- Pull latest telemetry, closest-approach actuals, splashdown conditions
- Update component data with any post-flight revelations

## How to resume

When starting a new conversation, tell Claude:
1. "Read `/mnt/user-data/outputs/PROJECT_STATE.md` first"
2. "The working file is `/mnt/user-data/outputs/artemis2-3d-simulator.jsx`"
3. "Source parts are in `/home/claude/artemis_part{1,2,3}.txt` if they persist, otherwise extract from the .jsx"
4. "Pick task N from the TODO list"

If `/home/claude/` has been cleared between sessions, the single .jsx file in outputs is the source of truth — split it back into parts at comment markers like `/* ─── PHASE METADATA */` for part1→part2 boundary and `export default function` for part2→part3 boundary.

## Constitutional principles for this project
- SI units always (Zoltan preference)
- No emojis (Zoltan preference)
- Verified data only — mark unknowns as "not officially published"
- Font sizes 8/10/12 baseline
- Trajectory sync via single source of truth (no duplicated geometry)
- Do not break existing working physics when adding features
- Test against validate_sync.js before declaring done
