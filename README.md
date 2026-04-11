# Artemis II Simulator

High-fidelity 3D simulator of NASA's Artemis II lunar flyby mission, from launch through splashdown. Built with React + Three.js + Vite.

## Features
- 20 mission phases: SRB burn → HEO insertion → TLI → outbound → lunar flyby (~6,513 km) → return → atmospheric entry → splashdown
- Synchronized 3-body physics (Earth + Moon) with J2 perturbation
- RK4 integration for launch through TLI, unified `flybyPointAt()` geometry through encounter, Hermite return arc to entry interface
- Interactive 3D scene with Three.js, realistic SLS / Orion / entry-configuration models
- Component carousel with 12 verified vehicle parts (data sourced from NASA, ESA, Airbus, Lockheed Martin, Northrop Grumman)
- All 19 phases validated continuous against NASA Artemis II published values (closest approach 6,539 km vs NASA 6,513 km)

## Quick start
```bash
npm install
npm run dev       # open http://localhost:5173
npm run build     # production bundle in dist/
npm run validate  # run trajectory sanity tests
```

## Project structure
```
artemis2-sim/
├── src/
│   ├── main.jsx                  # React entry
│   └── Artemis2Simulator.jsx     # Main component (physics + 3D + UI)
├── scripts/
│   └── validate_trajectory.js    # Node-based sanity check
├── docs/
│   └── PROJECT_STATE.md          # Roadmap & resumption guide
├── .vscode/                      # VS Code debugger + extension hints
├── .github/workflows/ci.yml      # GitHub Actions build check
├── vite.config.js
└── package.json
```

## Roadmap
See `docs/PROJECT_STATE.md` for the queued TODO list (live NASA news panel, Lambert solver, astronaut bios, engine plume animations, post-launch data sync).

## Development with AI assistance
This project is optimized for VS Code with Claude Dev or GitHub Copilot. Recommended extensions auto-suggested via `.vscode/extensions.json`.

## License
MIT — see LICENSE.
