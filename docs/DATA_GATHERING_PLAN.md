# Artemis II Data Gathering Plan

A prioritized guide for collecting official mission data to improve the simulator's fidelity. All sources are publicly available.

---

## Priority 1 — Trajectory & Navigation (Highest Impact)

### 1.1 NASA Artemis II Mission Design Documents

**Where:**
- https://www.nasa.gov/artemis/artemis-2/
- NASA Technical Reports Server: https://ntrs.nasa.gov (search "Artemis II trajectory")
- NASA HORIZONS Web Interface: https://ssd.jpl.nasa.gov/horizons/

**What to grab:**
- [ ] Trajectory design overview PDF (published ~2022–2023)
- [ ] State vectors at key milestones (TLI, closest lunar approach, TEI if applicable)
- [ ] Closest lunar distance confirmation (target: 6,513 km from surface = 8,250 km from center)
- [ ] Mission elapsed time (MET) at each major event (MECO, TLI, flyby periapsis, reentry)
- [ ] Entry velocity (expected ~11.0 km/s), entry angle (nominal: -5.3°), splashdown target coordinates

**From HORIZONS:**
1. Go to https://ssd.jpl.nasa.gov/horizons/
2. Target body: `Orion` or search by NAIF ID once assigned post-launch
3. Observer location: `500@399` (geocenter)
4. Table settings: State vectors, ecliptic J2000
5. Output: Position + velocity every 1 hour from T+0 to T+10 days

---

## Priority 2 — Launch Vehicle Parameters (SLS Block 1)

### 2.1 SLS Vehicle Fact Sheet

**Where:** https://www.boeing.com/spacecraft/sls, https://www.nasa.gov/reference/space-launch-system/

**What to grab:**
- [ ] Core stage propellant mass (LOX 537,000 kg, LH2 90,760 kg — verify)
- [ ] SRB propellant mass each (~590,000 kg solid — verify)
- [ ] RS-25 thrust per engine (2,090 kN vacuum)
- [ ] Total SLS liftoff thrust (~39,000 kN)
- [ ] ICPS (Interim Cryogenic Propulsion Stage) Isp and RL-10C-1 thrust (~110 kN)
- [ ] Separation event times (SRB at T+128s, core MECO at T+486s)
- [ ] Vehicle mass breakdown (launch mass ~2,608,000 kg)

**Screenshot/save:** The SLS Block 1 infographic from NASA (`nasa.gov/image-detail/space-launch-system-block-1`) — use for 3D model proportions

### 2.2 Orion Capsule Parameters

**Where:** https://www.nasa.gov/orion/, ESA service module pages

**What to grab:**
- [ ] Orion bluntbody heat shield diameter (5.03 m)
- [ ] AJ10 service propulsion system: thrust 26.7 kN, Isp 316 s
- [ ] Crew module mass ~10,400 kg, service module ~15,053 kg
- [ ] Total Orion stack mass (crew + SM): ~26,520 kg
- [ ] Heat shield material: AVCOAT ablator
- [ ] Peak heating rate (second lunar return): ~1,700 W/cm²

---

## Priority 3 — Astronaut Crew Data

### 3.1 Crew Biographies

**Where:** https://www.nasa.gov/astronauts/

**Crew to document:**
| Astronaut | Role | Get from |
|-----------|------|---------|
| Reid Wiseman | Commander | nasa.gov/bio |
| Victor Glover | Pilot | nasa.gov/bio |
| Christina Hammock Koch | Mission Specialist 1 | nasa.gov/bio |
| Jeremy Hansen (CSA) | Mission Specialist 2 | asc-csa.gc.ca |

**What to grab per astronaut:**
- [ ] Official headshot (NASA public domain)
- [ ] Previous missions and flight hours
- [ ] Role on Artemis II (backup system responsibilities)
- [ ] Hometown / background

---

## Priority 4 — Mission Timeline Events

### 4.1 Official Flight Plan / Press Kit

**Where:** NASA Artemis II press kit (search "Artemis II Press Kit PDF" on ntrs.nasa.gov or nasa.gov/press-releases)

**What to grab:**
- [ ] Complete MET timeline (Mission Elapsed Time table)
- [ ] All planned burns: time, delta-V, duration
- [ ] SRB separation exact time (T+128s nominal)
- [ ] Core MECO time (T+486s nominal)
- [ ] ICPS separation time (~T+540s)
- [ ] TLI burn time and duration (~T+25.6h, ~349s)
- [ ] Closest lunar approach MET
- [ ] Entry interface MET and coordinates
- [ ] Splashdown target: Pacific Ocean, ~660 km NE of Maui

**How to read the timeline:** Times are in Day:Hour:Minute:Second from liftoff

---

## Priority 5 — Live Mission Data (Post-Launch)

### 5.1 NASA Eyes on the Solar System

**Where:** https://eyes.nasa.gov/apps/solar-system/

After launch (April 1, 2026 or later):
- [ ] Real trajectory SPICE kernels may be released
- [ ] Download tool: NAIF/SPICE toolkit at https://naif.jpl.nasa.gov
- [ ] Kernel file format: `.bsp` (binary SPK ephemeris)

### 5.2 NASA Space Flight RSS / API

- NASA API (APOD + general): https://api.nasa.gov (key required, free)
- [ ] Register for a free API key at api.nasa.gov
- [ ] Check NASA's open data portal: https://data.nasa.gov
- [ ] NASA TV livestream telemetry overlays (for burn times)

### 5.3 Celestrak / Space-Track

**Where:** https://celestrak.org, https://www.space-track.org

- [ ] TLE (Two-Line Elements) for Orion once in orbit
- [ ] Note: TLEs are less accurate for high-eccentricity/lunar trajectories; use HORIZONS state vectors instead

---

## Priority 6 — Scientific & Reference Data

### 6.1 Moon Topography

**Where:** https://lunar.gsfc.nasa.gov/lola.html (LRO LOLA instrument)

- [ ] LOLA digital elevation model (DEM) for accurate crater depths / mare elevations
- [ ] LOLA Quick Map: https://trek.nasa.gov/moon/
- [ ] Key features to note: South Pole-Aitken Basin depth, Mare Imbrium extent

### 6.2 Earth Imagery References

**Where:**
- NASA Visible Earth: https://visibleearth.nasa.gov (Blue Marble images, public domain)
- NASA WorldView: https://worldview.earthdata.nasa.gov

- [ ] Blue Marble Next Generation (monthly composites, 2048×1024 or 8192×4096)
- [ ] Earth city lights (Black Marble): https://earthobservatory.nasa.gov/features/NightLights
- [ ] Specular/water mask: https://visibleearth.nasa.gov/images/84341

---

## Priority 7 — Historical Analogue Data (Apollo)

### 7.1 Apollo 13 / Apollo 8 Free-Return Comparison

Artemis II uses a hybrid free-return trajectory similar to Apollo 13. Reference:
- [ ] Apollo 13 trajectory reconstruction: ntrs.nasa.gov search "Apollo 13 trajectory"
- [ ] Apollo 8 lunar flyby altitude: 112 km (much lower — different profile)
- [ ] Apollo lunar revisit document: "Apollo Experience Report — Guidance and Control" (NASA TN D-8285)

---

## How to Capture Data for Copilot

When you find official figures, provide them in this format for easy entry into the simulator:

```
SOURCE: [URL or document name]
EVENT: [e.g. TLI burn]
MET: [e.g. T+25:36:00]
VALUE: [e.g. delta-V = 3,122 m/s]
CONFIDENCE: [official / estimated / derived]
```

Screenshots of NASA diagrams should also be shared — I can extract numbers from figures.

---

## Quick Links Summary

| Resource | URL |
|---------|-----|
| Artemis II mission page | https://www.nasa.gov/artemis/artemis-2/ |
| SLS overview | https://www.nasa.gov/reference/space-launch-system/ |
| Orion overview | https://www.nasa.gov/orion/ |
| HORIZONS trajectory tool | https://ssd.jpl.nasa.gov/horizons/ |
| NASA NTRS (technical reports) | https://ntrs.nasa.gov |
| NASA astronaut bios | https://www.nasa.gov/astronauts/ |
| ESA service module | https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Orion_Service_Module |
| NASA API | https://api.nasa.gov |
| Moon Trek (LOLA data) | https://trek.nasa.gov/moon/ |
| Earth Blue Marble | https://visibleearth.nasa.gov |
| NASA Eyes | https://eyes.nasa.gov |
