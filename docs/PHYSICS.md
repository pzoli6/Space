# Artemis II Simulator — Physics Documentation

## 1. Coordinate System

All physics computations use SI-adjacent units: **km** for distance, **km/s** for velocity, **seconds** for time.

### Orbital Plane Axes
- **X** — points toward the initial Moon position at mission epoch (approximately 0° ecliptic longitude)
- **Y** — 90° ahead of X in the orbital/equatorial plane
- **Z** — out-of-plane (inclination axis); positive toward ecliptic north

### State Vector
```
state = [x, y, z, vx, vy, vz]   (km, km/s)
```

### Three.js Mapping
Because Three.js uses a right-handed Y-up coordinate system, physics coordinates are remapped:
```
Three.js x = physics x * SC
Three.js y = physics z * SC      ← orbital-plane Z becomes scene Y
Three.js z = physics y * SC      ← orbital-plane Y becomes scene Z
SC = 1 / 10000   (1 Three.js unit = 10,000 km)
```

---

## 2. Physical Constants

| Symbol | Value | Description |
|--------|-------|-------------|
| `MU_E` | 398600.4418 km³/s² | Earth gravitational parameter |
| `MU_M` | 4902.800118 km³/s² | Moon gravitational parameter |
| `R_E` | 6378.137 km | Earth equatorial radius (WGS-84) |
| `R_M` | 1737.4 km | Mean lunar radius |
| `D_EM` | 384399 km | Mean Earth–Moon distance |
| `J2_E` | 1.08263e-3 | Earth oblateness coefficient (second zonal harmonic) |
| `MOON_PERIOD` | 2360591.5 s | Sidereal lunar orbital period (27.322 days) |
| `MOON_INCL` | 0.0898 rad | Lunar orbital inclination (~5.145°) |

---

## 3. Equations of Motion (EOM)

### 3.1 Earth Gravity
Standard two-body Newtonian gravity from Earth:
$$\ddot{\mathbf{r}} = -\frac{\mu_E}{r^3}\,\mathbf{r}$$
where $r = \|\mathbf{r}\|$.

### 3.2 J2 Oblateness Perturbation (optional, flag `useJ2`)
The leading zonal harmonic of Earth's gravity field shortens trajectories near Earth:
$$\Delta \ddot{x} = \frac{3}{2} J_2 \mu_E \frac{R_E^2}{r^5}\left[\left(5\frac{z^2}{r^2} - 1\right)x\right]$$
$$\Delta \ddot{y} = \frac{3}{2} J_2 \mu_E \frac{R_E^2}{r^5}\left[\left(5\frac{z^2}{r^2} - 1\right)y\right]$$
$$\Delta \ddot{z} = \frac{3}{2} J_2 \mu_E \frac{R_E^2}{r^5}\left[\left(5\frac{z^2}{r^2} - 3\right)z\right]$$

### 3.3 Lunar Gravity (Three-Body Term)
The Moon's position at time $t$ is computed from its circular orbit ephemeris:
$$\mathbf{r}_M(t) = D_{EM} \begin{bmatrix} \cos(\omega_M t + \theta_0) \\ \sin(\omega_M t + \theta_0)\cos(i_M) \\ \sin(\omega_M t + \theta_0)\sin(i_M) \end{bmatrix}$$
where $\omega_M = 2\pi / T_M$ and $\theta_0$ is the initial Moon angle (`th0`).

The Moon's gravitational acceleration on the spacecraft:
$$\ddot{\mathbf{r}}_{\text{Moon}} = \mu_M\left[\frac{\mathbf{r}_M - \mathbf{r}}{|\mathbf{r}_M - \mathbf{r}|^3} - \frac{\mathbf{r}_M}{|\mathbf{r}_M|^3}\right]$$
(The second term is the indirect acceleration from the Moon on Earth's center, required to keep the frame inertial.)

### 3.4 Combined EOM
```
eom3D(th0, useJ2): (state, t) -> [dx, dy, dz, dvx, dvy, dvz]
```
Returns the complete time derivative of the state vector combining Earth gravity, J2, and lunar perturbation.

---

## 4. Numerical Integration

### 4th-Order Runge-Kutta (RK4)
```js
rk4(state, t, dt, f)
```
Standard 4th-order Runge-Kutta with fixed step size `dt`:
$$\mathbf{k}_1 = f(\mathbf{s}, t)$$
$$\mathbf{k}_2 = f\!\left(\mathbf{s} + \tfrac{dt}{2}\mathbf{k}_1,\; t + \tfrac{dt}{2}\right)$$
$$\mathbf{k}_3 = f\!\left(\mathbf{s} + \tfrac{dt}{2}\mathbf{k}_2,\; t + \tfrac{dt}{2}\right)$$
$$\mathbf{k}_4 = f\!\left(\mathbf{s} + dt\,\mathbf{k}_3,\; t + dt\right)$$
$$\mathbf{s}(t+dt) = \mathbf{s}(t) + \frac{dt}{6}\left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)$$

**Step sizes used:**
| Phase | dt |
|-------|----|
| HEO coast (Phase 8) | 60 s |
| TLI + outbound coast | 120 s |
| Near-Moon (SOI, flyby) | 30 s |
| Return coast | 90 s |

---

## 5. Mission Phase Physics

### Phase 1–3: SRB + Core Stage Ascent (Kinematic Model)
Phases 1–3 use a **parameterized kinematic model**, not differential equations. The trajectory is pre-computed as a gravity-turn arc using:
- Vertical rise to ~140 km altitude
- Pitch-over to ~8° FPA at MECO
- Polynomial altitude and downrange profiles fit to SLS mission data
- Velocity magnitude follows Tsiolkovsky rocket equation approximation

**Known limitation:** The handoff between the ascent model and the RK4 orbital integration at Phase 4 entry has a small positional gap (~20 km) because the two models start from slightly different points.

### Phase 4: Orion Deployment / Initial Orbit
Spacecraft immediately enters Low Earth Orbit at approximately:
- Altitude: 185 × 1800 km (initial parking HEO)
- Velocity: ~7.8 km/s at perigee

### Phase 5–6: Perigee Raise + HEO Coast
Small apogee adjustment burn puts spacecraft into HEO. RK4 integration with J2 enabled.

### Phase 7–8: Apogee Raise + High Earth Orbit
Final burn to raise apogee to ~70,000 km for optimal TLI geometry. Coast phase uses full 3-body EOM.

### Phase 9: Trans-Lunar Injection (TLI)
RL-10 burn on ICPS raises apogee to exceed lunar SOI (~66,100 km). ~349 second burn.
- Departure velocity: ~10.7 km/s (validated)
- Delta-V: ~3.1 km/s

### Phase 10: Outbound Coast
**Known limitation:** Phase 10 uses a **smoothstep-bulged linear interpolation** between TLI exit state and lunar SOI entry point rather than full Keplerian/3-body integration. This avoids numerical instability from the coarse initial conditions but introduces trajectory shape approximations. The true trajectory would follow a Keplerian ellipse perturbed slightly by the Moon.

### Phases 11–13: Lunar Flyby (Scripted Arc)
The lunar flyby geometry is computed by `flybyPointAt(tau)` — a single source of truth for all flyby positions. The function uses a **parametric Hermite arc** that:
1. Enters the lunar SOI at tau=0 on an approach trajectory
2. Passes closest approach (~6,513 km from lunar surface) at tau ≈ 0.5
3. Exits the SOI on a hyperbolic departure trajectory at tau=1

**Known limitation:** This is a scripted arc, not the result of a Lambert solver or numerical integration of the lunar gravity field. The periapsis altitude is enforced analytically.

**Validated:** Closest lunar approach = 6,539 km (NASA target: 6,513 km, error ~0.4%).

### Phase 14: Return Coast
**Known limitation:** Return coast uses a **cubic Hermite spline** from SOI exit to entry interface, parameterized by time. Not a Keplerian trajectory.

### Phases 15–19: Entry, Descent, Landing
Splashdown trajectory is parameterized: CM enters at ~11 km/s, decelerates through heating phase, deploys drogues at ~7,620 m, mains at ~3,048 m, splashdown at ~9 m/s.

---

## 6. Validated Trajectory Outputs

Run `npm run validate` to check:

| Metric | Computed | NASA Target | Error |
|--------|----------|-------------|-------|
| Closest Moon surface | 6,539 km | 6,513 km | ~0.4% |
| Max Earth distance | ~444,000 km | 405,500 km | ~10% (flyby bias) |
| TLI velocity | 10.69 km/s | ~10.7 km/s | <0.1% |
| Max position jump | 11,845 km | <20,000 km | pass |

---

## 7. Known Approximations and Future Work

| Approximation | Location | Improvement |
|--------------|----------|-------------|
| Scripted flyby arc | `flybyPointAt()` | Replace with Lambert solver |
| Outbound coast lerp | Phase 10 | Full RK4 3-body integration |
| Return coast spline | Phase 14 | Full RK4 3-body integration |
| Fixed Moon ephemeris | `moonPos3D()` | Use DE421 JPL ephemeris |
| Ascent kinematic model | Phases 1–3 | 6-DOF aerodynamics model |
| No solar radiation pressure | All phases | Add SRP perturbation |
| Circular Moon orbit | `moonPos3D()` | Add eccentricity (e=0.0549) |

---

## 8. Telemetry Calculations

### Speed
$v = \sqrt{v_x^2 + v_y^2 + v_z^2}$ km/s from state vector

### Altitude above Earth
$h_E = \|\mathbf{r}\| - R_E$ km

### Distance from Moon surface
$d_M = |\mathbf{r} - \mathbf{r}_M(t)| - R_M$ km

### Acceleration (g-force)
Computed numerically using adjacent trajectory points (±5 index):
$$g_{\text{force}} = \frac{|v_{i+5} - v_{i-5}|}{t_{i+5} - t_{i-5}} \cdot \frac{1000}{9.80665}$$
(Factor of 1000: km/s → m/s conversion)

### Phase Color Coding
Each of the 19 mission phases has a distinct color `PH[i].hex` used for trajectory line rendering. Phase boundaries are highlighted in the phase timeline bar.
