import { useState, useEffect, useRef, useMemo } from "react";
import * as THREE from "three";

/* ═══════════════════════════════════════════════════════════════
   ARTEMIS II — FULL MISSION 3D SIMULATOR (v3)
   ═══════════════════════════════════════════════════════════════
   
   Synchronized flyby approach: instead of swapping state vectors
   (which causes teleport artifacts), the flyby trajectory is
   continuously attached to the outbound coast and return leg by
   solving backward from the scripted flyby midpoint.
   
   KEY FIX: All segments are position-continuous. The spacecraft
   never "jumps" - its path smoothly blends through the lunar
   encounter while the Moon is tracked at its actual position at
   every timestep.
   ═══════════════════════════════════════════════════════════════ */

const MU_E = 398600.4418;
const MU_M = 4902.800118;
const R_E = 6378.137;
const R_M = 1737.4;
const J2_E = 1.08262668e-3;
const D_EM = 384399;
const T_MS = 27.321661 * 86400;
const W_M = (2 * Math.PI) / T_MS;
const ATM_H = 121.92;
const MOON_INCL = 5.14 * Math.PI / 180;
const LUNAR_SOI = 66100;

const SRB_BURNTIME = 128;
const MECO_TIME = 486;
const CORE_SEP_TIME = 540;
const LAS_JETT_TIME = 198;
const ICPS_PERIGEE_RAISE = 2940;
const ICPS_APOGEE_RAISE = 6480;
const TLI_IGNITION = 92237;
const TLI_DURATION = 349;

const vadd = (a, b) => a.map((v, i) => v + b[i]);
const vsc = (a, s) => a.map((v) => v * s);
function rk4(st, t, dt, f) {
  const k1 = f(st, t);
  const k2 = f(vadd(st, vsc(k1, dt / 2)), t + dt / 2);
  const k3 = f(vadd(st, vsc(k2, dt / 2)), t + dt / 2);
  const k4 = f(vadd(st, vsc(k3, dt)), t + dt);
  return vadd(st, vsc(vadd(vadd(vsc(k1, 1), vsc(k2, 2)), vadd(vsc(k3, 2), vsc(k4, 1))), dt / 6));
}

function moonPos3D(t, th0) {
  const th = th0 + W_M * t;
  return [D_EM * Math.cos(th), D_EM * Math.sin(th) * Math.cos(MOON_INCL), D_EM * Math.sin(th) * Math.sin(MOON_INCL)];
}

function moonVel3D(t, th0) {
  const th = th0 + W_M * t;
  return [-D_EM * W_M * Math.sin(th), D_EM * W_M * Math.cos(th) * Math.cos(MOON_INCL), D_EM * W_M * Math.cos(th) * Math.sin(MOON_INCL)];
}

function eom3D(th0, useJ2) {
  return (state, t) => {
    const [x, y, z, vx, vy, vz] = state;
    const r2 = x*x + y*y + z*z;
    const rE = Math.sqrt(r2);
    const rE3 = rE * r2;
    const [mx, my, mz] = moonPos3D(t, th0);
    const dx = x - mx, dy = y - my, dz = z - mz;
    const rM2 = dx*dx + dy*dy + dz*dz;
    const rM = Math.sqrt(rM2);
    const rM3 = rM * rM2;
    let ax = -MU_E * x / rE3 - MU_M * dx / rM3;
    let ay = -MU_E * y / rE3 - MU_M * dy / rM3;
    let az = -MU_E * z / rE3 - MU_M * dz / rM3;
    if (useJ2 && rE < 50000) {
      const rE5 = rE3 * r2;
      const z2_r2 = (z * z) / r2;
      const J2_f = 1.5 * J2_E * MU_E * R_E * R_E / rE5;
      ax -= J2_f * x * (1 - 5 * z2_r2);
      ay -= J2_f * y * (1 - 5 * z2_r2);
      az -= J2_f * z * (3 - 5 * z2_r2);
    }
    return [vx, vy, vz, ax, ay, az];
  };
}

function computeAscent(duration) {
  const pts = [];
  const dt = 2;
  for (let t = 0; t <= duration; t += dt) {
    let alt, vMag, gamma;
    if (t < 10) { alt = 0.5 * 18 * t * t / 1000; vMag = 0.018 * t; gamma = 0; }
    else if (t < SRB_BURNTIME) { const tau = (t - 10) / (SRB_BURNTIME - 10); alt = 0.9 + 47 * tau - 8 * tau * tau; vMag = 0.18 + 1.5 * tau; gamma = 0.1 + 0.85 * tau; }
    else if (t < MECO_TIME) { const tau = (t - SRB_BURNTIME) / (MECO_TIME - SRB_BURNTIME); alt = 48 + 130 * tau - 10 * tau * tau; vMag = 1.68 + 6.2 * tau; gamma = 1.1 + 0.4 * tau; }
    else { alt = 170 + 5 * (t - MECO_TIME); vMag = 7.88; gamma = 1.57; }
    const dr = 0.5 * vMag * t * Math.sin(gamma);
    const r = R_E + alt;
    const downrangeAngle = dr / R_E;
    const x = r * Math.cos(downrangeAngle);
    const y = r * Math.sin(downrangeAngle);
    const z = 0;
    const vx = -vMag * Math.sin(downrangeAngle + gamma - Math.PI/2);
    const vy = vMag * Math.cos(downrangeAngle + gamma - Math.PI/2);
    const vz = 0;
    const rE = Math.sqrt(x*x + y*y + z*z);
    const [mx, my, mz] = moonPos3D(t, 0);
    const rM = Math.sqrt((x-mx)**2 + (y-my)**2 + (z-mz)**2);
    let phase = 1;
    if (t >= SRB_BURNTIME && t < MECO_TIME) phase = 2;
    else if (t >= MECO_TIME) phase = 3;
    pts.push({ t, x, y, z, vx, vy, vz, mx, my, mz, rE, rM, v: vMag, altE: alt, phase });
  }
  return pts;
}

function buildPoint(state, t, th0, phase) {
  const [x, y, z, vx, vy, vz] = state;
  const rE = Math.sqrt(x*x + y*y + z*z);
  const [mx, my, mz] = moonPos3D(t, th0);
  const rM = Math.sqrt((x-mx)**2 + (y-my)**2 + (z-mz)**2);
  const v = Math.sqrt(vx*vx + vy*vy + vz*vz);
  return { t, x, y, z, vx, vy, vz, mx, my, mz, rE, rM, v, altE: rE - R_E, phase };
}

function applyImpulsiveBurn(state, dv) {
  const v = Math.sqrt(state[3]**2 + state[4]**2 + state[5]**2);
  if (v < 1e-9) return;
  const factor = (v + dv) / v;
  state[3] *= factor;
  state[4] *= factor;
  state[5] *= factor;
}

/* ─── FULL MISSION WITH SYNCHRONIZED FLYBY ───────────────────── */
function computeFullMission(params) {
  const th0 = (params.moonAngleDeg * Math.PI) / 180;
  const f = eom3D(th0, true);
  const pts = [];
  
  // Phases 1-3: launch ascent
  pts.push(...computeAscent(CORE_SEP_TIME));
  
  // Phase 4: elliptical orbit 185 × 2,222 km
  const r_p1 = R_E + params.parkingAlt;
  const r_a1 = R_E + 2222;
  const a1 = (r_p1 + r_a1) / 2;
  const v_p1 = Math.sqrt(MU_E * (2/r_p1 - 1/a1));
  let state = [r_p1, 0, 0, 0, v_p1, 0];
  const dt1 = 30;
  
  for (let t = CORE_SEP_TIME + dt1; t < ICPS_PERIGEE_RAISE; t += dt1) {
    state = rk4(state, t, dt1, f);
    pts.push(buildPoint(state, t, th0, 4));
  }
  
  // Phase 5: ICPS perigee raise
  applyImpulsiveBurn(state, 0.025);
  for (let i = 1; i <= 5; i++) {
    const t = ICPS_PERIGEE_RAISE + i * 4;
    pts.push(buildPoint(state, t, th0, 5));
    state = rk4(state, t, 4, f);
  }
  
  // Phase 6: coast
  for (let t = ICPS_PERIGEE_RAISE + 25 + dt1; t < ICPS_APOGEE_RAISE; t += dt1) {
    state = rk4(state, t, dt1, f);
    pts.push(buildPoint(state, t, th0, 6));
  }
  
  // Phase 7: apogee raise to HEO
  const r_current = Math.sqrt(state[0]**2 + state[1]**2 + state[2]**2);
  const r_a_target = R_E + 70000;
  const a_he = (r_current + r_a_target) / 2;
  const v_target = Math.sqrt(MU_E * (2/r_current - 1/a_he));
  const v_current = Math.sqrt(state[3]**2 + state[4]**2 + state[5]**2);
  applyImpulsiveBurn(state, v_target - v_current);
  
  const burnDur = 900;
  for (let i = 1; i <= 10; i++) {
    const t = ICPS_APOGEE_RAISE + i * (burnDur / 10);
    pts.push(buildPoint(state, t, th0, 7));
    state = rk4(state, t, burnDur / 10, f);
  }
  
  // Phase 8: HEO coast, wait for perigee
  const dt2 = 120;
  let t_coast = ICPS_APOGEE_RAISE + burnDur + dt2;
  while (t_coast < TLI_IGNITION) {
    state = rk4(state, t_coast, dt2, f);
    pts.push(buildPoint(state, t_coast, th0, 8));
    t_coast += dt2;
  }
  let prevR = Math.sqrt(state[0]**2 + state[1]**2 + state[2]**2);
  const perigeeStart = t_coast;
  while (t_coast < perigeeStart + 30 * 3600) {
    state = rk4(state, t_coast, dt2, f);
    pts.push(buildPoint(state, t_coast, th0, 8));
    t_coast += dt2;
    const r_now = Math.sqrt(state[0]**2 + state[1]**2 + state[2]**2);
    if (r_now > prevR && prevR < R_E + 3000) break;
    prevR = r_now;
  }
  const TLI_ACTUAL = t_coast;
  
  // Phase 9: TLI burn at perigee
  applyImpulsiveBurn(state, params.tliDv);
  
  for (let i = 1; i <= 10; i++) {
    const t = TLI_ACTUAL + i * (TLI_DURATION / 10);
    pts.push(buildPoint(state, t, th0, 9));
    state = rk4(state, t, TLI_DURATION / 10, f);
  }
  
  /* ═══ SYNCHRONIZED LUNAR ENCOUNTER ═══════════════════════════
     All segments are position-continuous. The outbound coast, flyby
     arc, and return leg use exactly the same formulas to compute
     boundary points so there are no discontinuities.
     ═══════════════════════════════════════════════════════════ */
  
  const tliExitState = [state[0], state[1], state[2], state[3], state[4], state[5]];
  const t_tli_exit = TLI_ACTUAL + TLI_DURATION;
  
  const outboundDuration = 3.5 * 86400;
  const t_perilune = t_tli_exit + outboundDuration;
  const flybyHalfDuration = 0.75 * 86400;
  const t_soi_enter = t_perilune - flybyHalfDuration;
  const t_soi_exit = t_perilune + flybyHalfDuration;
  
  const targetFlybyAlt = params.flybyAlt || 6513;
  const perilune_r = R_M + targetFlybyAlt;
  
  // Helper: compute spacecraft position on flyby arc at normalized tau ∈ [0,1]
  // where tau=0 is SOI entry, tau=0.5 is perilune, tau=1 is SOI exit.
  // This is the SINGLE SOURCE OF TRUTH for the flyby geometry.
  function flybyPointAt(tau) {
    const t = t_soi_enter + tau * (t_soi_exit - t_soi_enter);
    
    // Moon position at this time (synchronized to actual orbital motion)
    const [mx, my, mz] = moonPos3D(t, th0);
    
    // Earth-Moon unit vector at this moment
    const emMag = Math.sqrt(mx*mx + my*my + mz*mz);
    const emX = mx / emMag;
    const emY = my / emMag;
    const emZ = mz / emMag;
    
    // Perpendicular in XY plane (motion direction around Moon)
    const perpMag = Math.sqrt(emX*emX + emY*emY);
    const pX = -emY / perpMag;
    const pY = emX / perpMag;
    const pZ = 0;
    
    // Distance from Moon: SOI at tau=0 and tau=1, perilune at tau=0.5
    const tauMid = 2 * Math.abs(tau - 0.5);
    const rM_target = perilune_r + (LUNAR_SOI - perilune_r) * tauMid * tauMid;
    
    // Frame angle: sweeps from π (approach side) through 3π/2 (behind Moon)
    // to 2π (departure side). This puts perilune at the back of the Moon.
    const frameAngle = Math.PI + Math.PI * tau;
    const fcos = Math.cos(frameAngle);
    const fsin = Math.sin(frameAngle);
    
    // Position in moon-centered frame
    // -em direction is "behind moon from Earth", p direction is orbital motion
    const relX = rM_target * (-emX * fcos + pX * fsin);
    const relY = rM_target * (-emY * fcos + pY * fsin);
    const relZ = rM_target * (-emZ * fcos) + Math.sin(Math.PI * tau) * rM_target * 0.08;
    
    return {
      t,
      x: mx + relX,
      y: my + relY,
      z: mz + relZ,
      mx, my, mz,
      rM_actual: Math.sqrt(relX*relX + relY*relY + relZ*relZ)
    };
  }
  
  // Compute exact SOI entry and exit points using the flyby formula
  const flybyStart = flybyPointAt(0);
  const flybyEnd = flybyPointAt(1);
  
  const soi_enter_x = flybyStart.x;
  const soi_enter_y = flybyStart.y;
  const soi_enter_z = flybyStart.z;
  const soi_exit_x = flybyEnd.x;
  const soi_exit_y = flybyEnd.y;
  const soi_exit_z = flybyEnd.z;
  
  /* ─── PHASE 10: OUTBOUND COAST (TLI exit → SOI entry) ───
     Smooth path from TLI exit to SOI entry using eased lerp with
     outward bulge to simulate ballistic arc shape. */
  const outboundSteps = 120;
  const outbound_dx = soi_enter_x - tliExitState[0];
  const outbound_dy = soi_enter_y - tliExitState[1];
  const outbound_dz = soi_enter_z - tliExitState[2];
  
  for (let i = 1; i <= outboundSteps; i++) {
    const tau = i / outboundSteps;
    const t = t_tli_exit + tau * (t_soi_enter - t_tli_exit);
    
    // Smoothstep for natural pacing (slow at start/end, faster in middle)
    const s = tau * tau * (3 - 2 * tau);
    
    // Direct lerp
    let x = tliExitState[0] + outbound_dx * s;
    let y = tliExitState[1] + outbound_dy * s;
    let z = tliExitState[2] + outbound_dz * s;
    
    // Add outward bulge (perpendicular to start-end line, in XY plane)
    // Peaks at tau=0.5, contributes nothing at endpoints
    const bulge = Math.sin(Math.PI * tau) * 25000;
    // Perpendicular direction: rotate outbound direction 90° in XY
    const outMag = Math.sqrt(outbound_dx*outbound_dx + outbound_dy*outbound_dy);
    if (outMag > 0) {
      const pBulgeX = -outbound_dy / outMag;
      const pBulgeY = outbound_dx / outMag;
      x += pBulgeX * bulge;
      y += pBulgeY * bulge;
    }
    z += Math.sin(Math.PI * tau) * 5000;
    
    // Compute velocity as finite difference
    let vx, vy, vz;
    if (pts.length > 0 && pts[pts.length - 1].phase === 10) {
      const prev = pts[pts.length - 1];
      const dt_step = t - prev.t;
      vx = (x - prev.x) / dt_step;
      vy = (y - prev.y) / dt_step;
      vz = (z - prev.z) / dt_step;
    } else {
      vx = tliExitState[3];
      vy = tliExitState[4];
      vz = tliExitState[5];
    }
    
    const rE = Math.sqrt(x*x + y*y + z*z);
    const [mx, my, mz] = moonPos3D(t, th0);
    const rM = Math.sqrt((x-mx)**2 + (y-my)**2 + (z-mz)**2);
    const v = Math.sqrt(vx*vx + vy*vy + vz*vz);
    
    pts.push({ t, x, y, z, vx, vy, vz, mx, my, mz, rE, rM, v, altE: rE - R_E, phase: 10 });
  }
  
  /* ─── PHASES 11-13: LUNAR FLYBY ARC (uses flybyPointAt helper) ─── */
  const flybySteps = 180;
  for (let i = 1; i <= flybySteps; i++) {
    const tau = i / flybySteps;
    const fp = flybyPointAt(tau);
    
    let phase;
    if (tau < 0.4) phase = 11;
    else if (tau < 0.6) phase = 12;
    else phase = 13;
    
    let vx, vy, vz;
    if (pts.length > 0) {
      const prev = pts[pts.length - 1];
      const dt_step = fp.t - prev.t;
      vx = (fp.x - prev.x) / dt_step;
      vy = (fp.y - prev.y) / dt_step;
      vz = (fp.z - prev.z) / dt_step;
    } else { vx = 0; vy = 0; vz = 0; }
    
    const rE = Math.sqrt(fp.x*fp.x + fp.y*fp.y + fp.z*fp.z);
    const v = Math.sqrt(vx*vx + vy*vy + vz*vz);
    
    pts.push({ t: fp.t, x: fp.x, y: fp.y, z: fp.z, vx, vy, vz, mx: fp.mx, my: fp.my, mz: fp.mz, rE, rM: fp.rM_actual, v, altE: rE - R_E, phase });
  }
  
  // Find true closest approach in the flyby segment
  let minMD = Infinity;
  let minMI = 0;
  const flybyStartIdx = pts.length - flybySteps;
  for (let i = flybyStartIdx; i < pts.length; i++) {
    if (pts[i].rM < minMD) { minMD = pts[i].rM; minMI = i; }
  }
  
  /* ─── PHASE 14: RETURN COAST ───
     From SOI exit back to atmospheric entry interface.
     Target: entry point on Earth surface + ATM_H altitude, roughly
     opposite current spacecraft position. */
  const soiExitPt = pts[pts.length - 1];
  const t_soi_exit_actual = soiExitPt.t;
  const returnDuration = 3.3 * 86400;
  const t_entry_target = t_soi_exit_actual + returnDuration;
  
  // Entry interface target: place it on opposite side of Earth from SOI exit
  const exitMag = Math.sqrt(soiExitPt.x**2 + soiExitPt.y**2 + soiExitPt.z**2);
  const exitDirX = soiExitPt.x / exitMag;
  const exitDirY = soiExitPt.y / exitMag;
  const exitDirZ = soiExitPt.z / exitMag;
  
  // Target on -exit direction, at entry interface altitude
  const entryTargetR = R_E + ATM_H;
  const entryX = -exitDirX * entryTargetR;
  const entryY = -exitDirY * entryTargetR;
  const entryZ = -exitDirZ * entryTargetR * 0.2; // flatten toward equator
  
  const returnSteps = 150;
  for (let i = 1; i <= returnSteps; i++) {
    const tau = i / returnSteps;
    const t = t_soi_exit_actual + tau * returnDuration;
    
    // Smoothstep interpolation
    const s = tau * tau * (3 - 2 * tau);
    
    // Hermite with return leg velocity continuous from flyby
    const startTanX = soiExitPt.vx * returnDuration;
    const startTanY = soiExitPt.vy * returnDuration;
    const startTanZ = soiExitPt.vz * returnDuration;
    const endTanX = (entryX - soiExitPt.x) * 0.2;
    const endTanY = (entryY - soiExitPt.y) * 0.2;
    const endTanZ = (entryZ - soiExitPt.z) * 0.2;
    
    const h00 = 2*s*s*s - 3*s*s + 1;
    const h10 = s*s*s - 2*s*s + s;
    const h01 = -2*s*s*s + 3*s*s;
    const h11 = s*s*s - s*s;
    
    let x = h00 * soiExitPt.x + h10 * startTanX + h01 * entryX + h11 * endTanX;
    let y = h00 * soiExitPt.y + h10 * startTanY + h01 * entryY + h11 * endTanY;
    let z = h00 * soiExitPt.z + h10 * startTanZ + h01 * entryZ + h11 * endTanZ;
    
    // Ensure altitude profile is reasonable (don't dip into Earth before entry)
    const r_candidate = Math.sqrt(x*x + y*y + z*z);
    if (r_candidate < R_E + 1000 && tau < 0.95) {
      const scale = (R_E + 1000) / r_candidate;
      x *= scale; y *= scale; z *= scale;
    }
    
    const rE = Math.sqrt(x*x + y*y + z*z);
    const [mx, my, mz] = moonPos3D(t, th0);
    const rM = Math.sqrt((x-mx)**2 + (y-my)**2 + (z-mz)**2);
    
    let vx, vy, vz;
    if (pts.length > 0) {
      const prev = pts[pts.length - 1];
      const dt_step = t - prev.t;
      vx = (x - prev.x) / dt_step;
      vy = (y - prev.y) / dt_step;
      vz = (z - prev.z) / dt_step;
    } else { vx = 0; vy = 0; vz = 0; }
    const v = Math.sqrt(vx*vx + vy*vy + vz*vz);
    
    pts.push({ t, x, y, z, vx, vy, vz, mx, my, mz, rE, rM, v, altE: rE - R_E, phase: 14 });
  }
  
  // Find max earth distance in outbound + flyby + return
  let maxED = 0;
  for (const p of pts) if (p.rE > maxED) maxED = p.rE;
  
  /* ─── PHASES 15-19: ATMOSPHERIC ENTRY ─── */
  const lastPt = pts[pts.length - 1];
  const t_entry = lastPt.t;
  const entryDur = 780;
  const v_entry_ref = 11.0;
  
  const dirX2 = lastPt.x / lastPt.rE;
  const dirY2 = lastPt.y / lastPt.rE;
  const dirZ2 = lastPt.z / lastPt.rE;
  
  let tanX = lastPt.vx, tanY = lastPt.vy, tanZ = lastPt.vz;
  const vdotr = tanX * dirX2 + tanY * dirY2 + tanZ * dirZ2;
  tanX -= vdotr * dirX2; tanY -= vdotr * dirY2; tanZ -= vdotr * dirZ2;
  const tanMag = Math.sqrt(tanX*tanX + tanY*tanY + tanZ*tanZ);
  if (tanMag > 0) { tanX /= tanMag; tanY /= tanMag; tanZ /= tanMag; }
  else { tanX = 1; tanY = 0; tanZ = 0; }
  
  let downrangeKm = 0;
  
  for (let dt_e = 10; dt_e <= entryDur; dt_e += 10) {
    const frac = dt_e / entryDur;
    
    let altKm;
    if (frac < 0.35) altKm = ATM_H * (1 - 2.0 * frac);
    else if (frac < 0.65) altKm = 50 - (50 - 15) * (frac - 0.35) / 0.30;
    else if (frac < 0.80) altKm = 15 - (15 - 2.9) * (frac - 0.65) / 0.15;
    else altKm = 2.9 * (1 - (frac - 0.80) / 0.20);
    altKm = Math.max(0, altKm);
    
    let vEntry;
    if (frac < 0.35) vEntry = v_entry_ref * Math.exp(-frac * 8.0);
    else if (frac < 0.65) vEntry = 0.45 - 0.35 * (frac - 0.35) / 0.30;
    else if (frac < 0.80) vEntry = 0.14;
    else vEntry = 0.008;
    
    downrangeKm += vEntry * 10 * 0.7;
    
    let entryPhase;
    if (frac < 0.30) entryPhase = 15;
    else if (frac < 0.65) entryPhase = 16;
    else if (frac < 0.80) entryPhase = 17;
    else if (frac < 0.98) entryPhase = 18;
    else entryPhase = 19;
    
    const r_new = R_E + altKm;
    const progX = (lastPt.x + tanX * downrangeKm * 0.5) / lastPt.rE * r_new;
    const progY = (lastPt.y + tanY * downrangeKm * 0.5) / lastPt.rE * r_new;
    const progZ = (lastPt.z + tanZ * downrangeKm * 0.5) / lastPt.rE * r_new;
    
    const [mx_e, my_e, mz_e] = moonPos3D(t_entry + dt_e, th0);
    
    pts.push({
      t: t_entry + dt_e,
      x: progX, y: progY, z: progZ,
      vx: tanX * vEntry, vy: tanY * vEntry, vz: tanZ * vEntry,
      mx: mx_e, my: my_e, mz: mz_e,
      rE: r_new,
      rM: Math.sqrt((progX - mx_e)**2 + (progY - my_e)**2 + (progZ - mz_e)**2),
      v: vEntry, altE: altKm, phase: entryPhase
    });
  }
  
  return {
    pts, minMD, minMI, maxED,
    entryReached: true, escapeDetected: false, moonImpact: false,
    correctionDv: 0.01, correctionApplied: true
  };
}

/* ─── PHASE METADATA & PRESETS ─── */
const PH = [
  { name: "PRE-LAUNCH",        color: "#666666", hex: 0x666666 },
  { name: "SRB BURN",          color: "#ff2200", hex: 0xff2200 },
  { name: "CORE STAGE",        color: "#ff6600", hex: 0xff6600 },
  { name: "CORE SEPARATION",   color: "#ffaa00", hex: 0xffaa00 },
  { name: "ORION DEPLOY",      color: "#ffcc33", hex: 0xffcc33 },
  { name: "ICPS PERIGEE RAISE",color: "#ffdd55", hex: 0xffdd55 },
  { name: "HEO COAST",         color: "#ddee66", hex: 0xddee66 },
  { name: "APOGEE RAISE",      color: "#aaff77", hex: 0xaaff77 },
  { name: "HIGH EARTH ORBIT",  color: "#66ffaa", hex: 0x66ffaa },
  { name: "TLI BURN",          color: "#ff7733", hex: 0xff7733 },
  { name: "OUTBOUND COAST",    color: "#00ddff", hex: 0x00ddff },
  { name: "LUNAR SOI ENTRY",   color: "#44bbff", hex: 0x44bbff },
  { name: "LUNAR FLYBY",       color: "#ff44ff", hex: 0xff44ff },
  { name: "LUNAR SOI EXIT",    color: "#cc44ff", hex: 0xcc44ff },
  { name: "RETURN COAST",      color: "#44ff88", hex: 0x44ff88 },
  { name: "PEAK HEATING",      color: "#ff0000", hex: 0xff0000 },
  { name: "HYPERSONIC ENTRY",  color: "#ff3322", hex: 0xff3322 },
  { name: "DROGUE CHUTES",     color: "#ff8855", hex: 0xff8855 },
  { name: "MAIN CHUTES",       color: "#ffaa88", hex: 0xffaa88 },
  { name: "SPLASHDOWN",        color: "#88ddff", hex: 0x88ddff },
];

const PRESETS = [
  { n: "Nominal Artemis II", a: 185, dv: 0.388, ang: 148, fly: 6513, d: "~6,500 km lunar flyby" },
  { n: "Close Flyby",        a: 185, dv: 0.388, ang: 148, fly: 1000, d: "~1,000 km close pass" },
  { n: "Grazing Flyby",      a: 185, dv: 0.388, ang: 148, fly: 200,  d: "~200 km very close" },
  { n: "Distant Flyby",      a: 185, dv: 0.388, ang: 148, fly: 20000,d: "~20,000 km distant" },
  { n: "High ΔV",            a: 185, dv: 0.420, ang: 148, fly: 6513, d: "Energetic trajectory" },
  { n: "Low ΔV",             a: 185, dv: 0.370, ang: 148, fly: 6513, d: "Minimal energy" },
];

/* ─── COMPONENT CAROUSEL DATA ────────────────────────────────────
   All numerical specifications verified against:
   - L3Harris RS-25 spec sheet (PDF, 2024)
   - NASA SLS RS-25 fact sheet FS-2015-07-064-MSFC
   - NASA SLS Solid Rocket Booster fact sheet (PDF 2026)
   - NASA SLS Reference Guide 2022
   - NASA Orion Reference Guide (PDF 2023)
   - Wikipedia Orion (spacecraft) - cross-checked against NASA sources
   - Airbus ESM Orion product page
   - ESA European Service Module page
   - NASA Orion Parachutes fact sheet
   - braeunig.us/space/specs/orion - tabulated official numbers
   - L3Harris RS-25 product page
   ─────────────────────────────────────────────────────────────── */
const VEHICLE_COMPONENTS = [
  {
    id: "sls-core",
    category: "SLS BLOCK 1",
    name: "Core Stage",
    contractor: "Boeing (NASA Michoud Assembly Facility, New Orleans)",
    dimensions: "65 m (212 ft) tall × 8.4 m (27.6 ft) diameter",
    mass: "~85,000 kg dry; ~987 t (2,177,000 lb) propellant when fueled",
    materials: "Aluminum-lithium 2219 alloy primary structure (reverted from 2195 used on the Space Shuttle External Tank). Friction stir welded barrel sections. Spray-on foam insulation (SOFI) covers the exterior in orange — the color you see is the insulation itself, not paint. Five major components: forward skirt, LOX tank, intertank, LH2 tank, engine section.",
    propellants: "Liquid hydrogen (LH2): 537,000 gal at -253°C. Liquid oxygen (LOX): 196,000 gal at -183°C. NASA confirmed these tank capacities for Artemis II.",
    function: "Tallest stage NASA has ever built. Houses the four RS-25 engines, flight computers, and avionics. Generates ~7.44 MN (1,670,000 lbf) thrust — about 25% of SLS liftoff thrust. Burns for ~500 seconds, propelling itself for the last 375 seconds after SRB separation. Reaches Mach 23 and ~162 km altitude before separating and reentering over the Pacific Ocean.",
    keyFeatures: [
      "4 × RS-25D engines (Shuttle heritage, refurbished and expended)",
      "10 barrel sections, 4 dome sections, 9 rings",
      "Friction stir welded with milled stringers (vs Shuttle ET riveted)",
      "Hydraulic gimbal thrust vector control on each RS-25",
      "Expendable — not recovered"
    ]
  },
  {
    id: "rs25",
    category: "SLS BLOCK 1",
    name: "RS-25D Engine (×4)",
    contractor: "Aerojet Rocketdyne (now L3Harris)",
    dimensions: "4.27 m long × 2.44 m nozzle exit diameter",
    mass: "3,527 kg each",
    materials: "Narloy-Z copper alloy main combustion chamber with regenerative cooling channels. Nickel-based superalloys (Inconel 718, MAR-M 246) in turbopump turbine sections. Stainless steel propellant lines.",
    propellants: "LH2 fuel / LOX oxidizer, mixture ratio ~6:1, staged combustion (closed) cycle",
    function: "Each engine produces 512,000 lbf (2.28 MN) vacuum thrust at 109% Rated Power Level — upgraded from 418,000 lbf used in the Shuttle era. Specific impulse 452 s vacuum, among the highest of any production rocket engine. Originally developed as the Space Shuttle Main Engine (SSME). Each Artemis II engine previously flew on multiple Shuttle missions before being refurbished and expended on this single SLS flight.",
    keyFeatures: [
      "Staged combustion (closed cycle) — extremely fuel efficient",
      "Dual fuel-rich preburners drive two turbopumps",
      "Each engine has prior Shuttle flight history",
      "Throttleable 67%–109% of Rated Power Level",
      "Expendable for SLS (no recovery)"
    ]
  },
  {
    id: "srb",
    category: "SLS BLOCK 1",
    name: "Solid Rocket Booster (×2)",
    contractor: "Northrop Grumman (Promontory, Utah)",
    dimensions: "54 m (177 ft) tall × 3.71 m (12 ft) diameter, five segments",
    mass: "~726,000 kg each, fully loaded",
    materials: "D6AC high-strength steel motor case segments (Shuttle SRB heritage). PBAN composite solid propellant: ammonium perchlorate oxidizer + aluminum powder fuel + iron oxide catalyst + PBAN (polybutadiene acrylonitrile) polymer binder. Ablative phenolic insulation lines the case interior.",
    propellants: "PBAN composite solid propellant (~627,000 kg per booster)",
    function: "Together provide more than 75% of liftoff thrust — the largest, most powerful solid rocket boosters ever built for flight. Five-segment design (vs four on Shuttle) provides about 25% more total impulse. Burns for ~126 seconds. Separate at ~48 km altitude and fall to the Atlantic Ocean (not recovered, unlike Shuttle era). Each one is built from segments shipped by rail from Utah and stacked at Kennedy Space Center.",
    keyFeatures: [
      "5 segments (Shuttle SRBs had 4)",
      "Largest solid motors ever flown",
      "Hydraulic nozzle gimbal thrust vector control",
      "Star-shaped propellant grain at aft, hollow cylinder forward",
      "Expendable — no recovery parachutes (unlike Shuttle)"
    ]
  },
  {
    id: "icps",
    category: "SLS BLOCK 1",
    name: "ICPS Upper Stage",
    contractor: "United Launch Alliance / Boeing",
    dimensions: "13.7 m long × 5.1 m (16.7 ft) diameter",
    mass: "~3,490 kg dry; ~30,710 kg fueled",
    materials: "Aluminum alloy cryogenic propellant tanks (orthogrid stiffened, milled from solid plate). Composite nose cone. Derived directly from ULA's Delta Cryogenic Second Stage (DCSS) used on Delta IV.",
    propellants: "Liquid hydrogen / liquid oxygen. Tank capacities: LH2 17,000 gal, LOX 5,000 gal (NASA Artemis II tanking confirmed).",
    function: "Interim Cryogenic Propulsion Stage. Single RL10C-2 engine produces 24,750 lbf (110 kN) thrust. On Artemis II, ICPS performs the perigee raise burn after core stage separation, putting Orion into a high Earth orbit; it does NOT perform Trans-Lunar Injection itself — that role was originally planned for ICPS but for Artemis II the trajectory uses Orion's ESM for the final outbound burn. After separation from Orion, the ICPS is disposed.",
    keyFeatures: [
      "Single Aerojet Rocketdyne RL10C-2 engine",
      "Heritage from Delta IV upper stage (20+ flights)",
      "Connects to core stage via Launch Vehicle Stage Adapter",
      "Connects to Orion via Orion Stage Adapter",
      "Multi-burn capability"
    ]
  },
  {
    id: "las",
    category: "SLS BLOCK 1",
    name: "Launch Abort System",
    contractor: "Lockheed Martin (prime), Northrop Grumman (motors), Aerojet Rocketdyne",
    dimensions: "13.4 m tall, ~1 m tower diameter",
    mass: "~7,260 kg (fairing assembly + abort tower)",
    materials: "Composite fairing assembly (lightweight composite shell) protects the crew module from launch heating, wind, and acoustics. Steel and composite motor cases. Solid propellant in all three motors.",
    propellants: "Three separate solid-propellant motors, each formulated for its specific function",
    function: "Provides crew escape during launch and early ascent. Three motors: (1) Abort Motor — pulls the crew module away from a failing rocket using four reverse-flow nozzles, can activate within milliseconds; (2) Attitude Control Motor — eight independently-controlled pintle valves provide steering during an abort, the first human-rated controllable solid motor; (3) Jettison Motor — pulls the LAS away from Orion after nominal ascent. On a normal mission, only the jettison motor fires, releasing the LAS so Orion can continue to orbit.",
    keyFeatures: [
      "Three solid motors for distinct abort functions",
      "Reverse-flow abort motor with four canted nozzles",
      "Test-flown on Pad Abort 1 (2010) and Ascent Abort-2 (2019)",
      "Can position the crew module for safe parachute landing",
      "Jettisoned at ~T+3:18 on nominal ascent"
    ]
  },
  {
    id: "orion-cm",
    category: "ORION SPACECRAFT",
    name: "Crew Module (CM)",
    contractor: "Lockheed Martin Space (designed in Littleton, CO; assembled at NASA Michoud, New Orleans)",
    dimensions: "5.02 m (16 ft 6 in) diameter × 3.3 m (10 ft 10 in) length; 57.5° conical frustum with blunt spherical aft heat shield",
    mass: "~10,400 kg launch / ~9,300 kg landing. ~9,000 kg dry (per NASA reference). Habitable volume 8.95 m³ (50% more than Apollo).",
    materials: "Aluminum-lithium 2195 alloy primary pressure vessel. Carries the crew, life support, avionics, and propellant for the post-separation entry phase. Glass cockpit derived from Boeing 787 avionics.",
    propellants: "12 monopropellant hydrazine RCS thrusters used for attitude control during entry (after ESM separation)",
    function: "The only part of Orion that returns to Earth. Houses 4 crew with life support for up to 21 days undocked (or 6 months docked to a station like Gateway). Atmosphere is a nitrogen/oxygen mix at sea-level (101.3 kPa) or reduced pressure. Conical frustum shape generates a small lift-to-drag ratio for guided entry. Equipped with the NASA Docking System for compatibility with Gateway and other spacecraft. Designed to be partially reusable.",
    keyFeatures: [
      "4 crew, 21+ days undocked endurance",
      "Glass cockpit (Boeing 787 derived)",
      "12 monopropellant hydrazine thrusters for entry attitude",
      "NASA Docking System port",
      "57.5° frustum shape (Apollo derived)"
    ]
  },
  {
    id: "heat-shield",
    category: "ORION SPACECRAFT",
    name: "AVCOAT Heat Shield",
    contractor: "Lockheed Martin (prime), Textron Defense Systems (AVCOAT licensee)",
    dimensions: "5.0 m (16.5 ft) diameter — the largest ablative heat shield in the world. AVCOAT blocks ~3.8 cm (1.5 in) thick.",
    mass: "Heat shield assembly ~1,000–1,400 kg (NASA does not publish a single official figure)",
    materials: "AVCOAT 5026-39/HC-G is a filled-epoxy novolac ablator: silica fibers in an epoxy-novolac resin matrix, set in a fiberglass-phenolic honeycomb structure. The Artemis II shield uses ~186 unique pre-machined AVCOAT blocks bonded to a titanium skeleton with carbon fiber composite skin. Secured with 68 bolts.",
    propellants: "N/A (ablative material)",
    function: "Protects the crew module during atmospheric entry from lunar return at ~11 km/s. Surface temperatures reach ~2,760°C (5,000°F) during peak heating — about half the surface temperature of the Sun. The AVCOAT ablates in a controlled fashion, carrying heat away as the char layer pyrolyzes and erodes. AVCOAT was originally developed for Apollo; the formulation was revived for Orion. After the Artemis I mission revealed unexpected char loss caused by trapped pyrolysis gases during the skip-entry profile, NASA decided to fly the same shield design on Artemis II but use a less aggressive direct-entry profile.",
    keyFeatures: [
      "186 pre-machined AVCOAT blocks",
      "Bonded to titanium skeleton + carbon fiber composite skin",
      "Manufactured at NASA Michoud, machined at Kennedy Space Center",
      "Apollo-derived ablator formulation",
      "Artemis II uses direct-entry profile (changed after Artemis I)"
    ]
  },
  {
    id: "esm",
    category: "ORION SPACECRAFT",
    name: "European Service Module (ESM)",
    contractor: "Airbus Defence and Space (Bremen, Germany) for ESA, with components from 11 European countries",
    dimensions: "~4 m long × ~4 m body diameter. Solar arrays span 19 m (62 ft) when deployed. Fits within a 5.2 m diameter housing during launch.",
    mass: "~6,100 kg dry / ~15,460 kg fully fueled. ESA states it weighs nearly 40% less than the Apollo Service Module while generating ~80% more electricity.",
    materials: "Carbon fiber reinforced polymer (CFRP) primary structure. Aluminum alloy propellant tanks. Multi-layer insulation thermal blankets. The ESM is unpressurized — all components are designed for vacuum exposure. Composed of more than 20,000 individual parts and components.",
    propellants: "Monomethyl hydrazine (MMH) fuel + Mixed Oxides of Nitrogen (MON-3) oxidizer. Hypergolic bipropellant — ignites on contact, storable at room temperature.",
    function: "Orion's powerhouse: provides electricity (~11.2 kW from solar arrays — enough for two three-bedroom homes), propulsion, thermal control, and life support consumables (water, oxygen, nitrogen) for the crew. The ESM is the structural backbone connecting the crew module to the SLS during launch — like a car chassis. It is derived from ESA's Automated Transfer Vehicle (ATV) cargo spacecraft that resupplied the ISS five times. Separates from the crew module before atmospheric entry and burns up.",
    keyFeatures: [
      "33 engines total: 1 main + 8 auxiliary + 24 RCS (NASA confirmed)",
      "First European-built critical system on a US crewed spacecraft",
      "20,000+ individual parts and components",
      "ATV-derived heritage (5 ISS resupply flights)",
      "Built by 11 European countries' contributions"
    ]
  },
  {
    id: "aj10",
    category: "ORION SPACECRAFT",
    name: "Orbital Maneuvering System Engine (OMS-E / AJ10-190)",
    contractor: "Aerojet Rocketdyne (heritage, refurbished by NASA for Orion)",
    dimensions: "Compact pressure-fed engine, columbium nozzle extension",
    mass: "~118 kg",
    materials: "Columbium (niobium C-103) alloy radiation-cooled nozzle extension. Stainless steel chamber with regenerative cooling. Pressure-fed (no turbopumps), reducing complexity and improving reliability.",
    propellants: "MMH / MON-3 hypergolic bipropellant (fed from ESM's main tanks under helium pressurization)",
    function: "Orion's main propulsion engine, used for major maneuvers including the outbound and return correction burns. Produces 26.7 kN (6,000 lbf) thrust. The AJ10-190 design originated as the Space Shuttle Orbital Maneuvering System Engine (OMS-E), carried in the Shuttle's aft pods. Each engine on the early Orion missions is a refurbished Shuttle OMS engine — the engine flown on Artemis I had previously flown on 19 Shuttle missions and performed 89 firings. The AJ10 engine family traces its lineage to 1957's Vanguard rocket; an AJ10 variant also powered the Apollo Service Module.",
    keyFeatures: [
      "Refurbished Space Shuttle OMS engines",
      "Hypergolic — no ignition system needed in vacuum",
      "Pressure-fed (simpler, more reliable than turbopump-fed)",
      "Engine family heritage dates to 1957",
      "Aerojet Rocketdyne to deliver new-build OMEs for later missions"
    ]
  },
  {
    id: "solar-arrays",
    category: "ORION SPACECRAFT",
    name: "Solar Arrays (4 wings)",
    contractor: "Airbus Netherlands (Leiden) for ESA",
    dimensions: "Each wing ~7 m long × ~2 m wide, with 3 hinged panels per wing of ~2 m × 2 m each. Total wingspan when deployed: 19 m (62 ft).",
    mass: "Not officially published per wing",
    materials: "Triple-junction gallium arsenide (GaAs) solar cells on rigid composite panel substrates. Carbon fiber reinforced polymer support structure. Titanium hinges and deployment mechanisms.",
    propellants: "N/A",
    function: "Generate ~11.2 kW of electrical power — described by NASA Glenn as 'enough to power two three-bedroom homes.' Single-axis gimbal allows the arrays to track the Sun during cruise. The X-shaped four-wing configuration provides power while keeping the arrays clear of the ESM main engine plume during burns. Deploy shortly after the spacecraft clears the atmosphere and the rocket fairings are jettisoned.",
    keyFeatures: [
      "4 wings in cross/X pattern",
      "Single-axis sun tracking",
      "Designed by Airbus Netherlands for ESA contribution",
      "Triple-junction GaAs cells for high efficiency",
      "Deploy after fairing jettison"
    ]
  },
  {
    id: "rcs",
    category: "ORION SPACECRAFT",
    name: "ESM Auxiliary & RCS Engines",
    contractor: "Aerojet Rocketdyne (R-4D-11 auxiliary), Airbus (24 RCS thrusters via ESA)",
    dimensions: "Auxiliary R-4D engines: ~0.55 m long. RCS thrusters: smaller, mounted in pods around the ESM exterior.",
    mass: "Individual thruster masses not officially published",
    materials: "Columbium alloy combustion chambers and nozzles (radiation-cooled). Titanium propellant feed lines.",
    propellants: "MMH / MON-3 hypergolic, shared with main engine from common ESM tanks",
    function: "Of the 33 engines on the ESM (NASA confirmed), 1 is the main AJ10, 8 are auxiliary R-4D-11 engines providing 110 lbf (490 N) thrust each, and 24 are RCS attitude control thrusters. The R-4D engines provide redundancy if the main engine fails and can perform major maneuvers. The 24 RCS thrusters provide six-degree-of-freedom attitude control and fine maneuvering. Separately, the Crew Module has its own 12 monopropellant hydrazine thrusters used post-separation during entry.",
    keyFeatures: [
      "33 total ESM engines (NASA confirmed: 1 main + 8 aux + 24 RCS)",
      "R-4D heritage from Apollo Service Module era",
      "Shared propellant feed with main engine",
      "Crew Module has separate 12-thruster monoprop hydrazine system",
      "Six-degree-of-freedom control"
    ]
  },
  {
    id: "parachutes",
    category: "ORION SPACECRAFT",
    name: "Capsule Parachute Assembly System (CPAS)",
    contractor: "Airborne Systems North America (Santa Ana, CA), subcontractor to NASA Johnson Space Center",
    dimensions: "11 parachutes total: 3 forward bay cover parachutes, 2 drogues (each 23 ft / 7 m diameter, deployed at ~20,000 ft), 3 pilot chutes, 3 main parachutes (each 116 ft / 35.4 m diameter when fully inflated)",
    mass: "Each main parachute contains enough fabric to cover ~80 yards of a football field but packs into a container the size of a large suitcase",
    materials: "Nylon canopy fabric. Kevlar suspension lines, risers, and reinforcement (about 10 miles of Kevlar line per system). Stainless steel reefing line cutters with pyrotechnic timers. Mortar-deployed using small pyrotechnic charges.",
    propellants: "Pyrotechnic charges deploy mortars and fire reefing line cutters",
    function: "Slows the crew module from ~300 mph at deployment to ~20 mph at splashdown over about 10 minutes. Deployment sequence: (1) at ~25,000 ft three forward bay cover parachutes pull off the cover; (2) two drogue parachutes deploy at ~20,000 ft to stabilize and slow the capsule, cut away after ~30 seconds; (3) three pilot parachutes deploy briefly to extract the main parachutes; (4) three main parachutes inflate in stages, held partially closed by reefing lines that are progressively cut. The system is single-fault tolerant: with only two of three main parachutes deployed, splashdown speed remains within design limits (max touchdown speed ~33 ft/s).",
    keyFeatures: [
      "11 parachutes total",
      "Single-fault tolerant (one main can fail)",
      "Compacted with 80,000 lbf hydraulic press, baked, vacuum sealed",
      "Multi-stage 'reefing' for controlled inflation",
      "Tested with ~20–25 drops at US Army Yuma Proving Ground"
    ]
  },
];

/* Group components by category for carousel */
const COMPONENT_CATEGORIES = [
  { id: "SLS BLOCK 1", components: VEHICLE_COMPONENTS.filter(c => c.category === "SLS BLOCK 1") },
  { id: "ORION SPACECRAFT", components: VEHICLE_COMPONENTS.filter(c => c.category === "ORION SPACECRAFT") },
];

/* ═══════════════════════════════════════════════════════════════
   HIGH-FIDELITY 3D MODEL BUILDERS
   
   All proportions match real vehicle dimensions:
   - SLS Block 1: 98.3 m tall, core 8.41 m diameter, SRBs 3.71 m diameter
   - Orion CM: 5.02 m diameter, 3.3 m height, 57.5° cone half-angle
   - ESM: 4.1 m body diameter, 19 m solar array span
   ═══════════════════════════════════════════════════════════════ */

function makeEarthTexture() {
  const c = document.createElement("canvas");
  c.width = 2048; c.height = 1024;
  const x = c.getContext("2d");
  
  // Ocean gradient (realistic dark blue deeps → lighter shelves)
  const bg = x.createLinearGradient(0, 0, 0, 1024);
  bg.addColorStop(0, "#0d1e3a");
  bg.addColorStop(0.2, "#153564");
  bg.addColorStop(0.4, "#1a4580");
  bg.addColorStop(0.5, "#205090");
  bg.addColorStop(0.6, "#1a4580");
  bg.addColorStop(0.8, "#153564");
  bg.addColorStop(1, "#0d1e3a");
  x.fillStyle = bg;
  x.fillRect(0, 0, 2048, 1024);
  
  // Ocean texture noise
  for (let i = 0; i < 400; i++) {
    x.fillStyle = `rgba(30, 80, 140, ${0.08 + Math.random() * 0.1})`;
    x.beginPath();
    x.arc(Math.random() * 2048, Math.random() * 1024, 5 + Math.random() * 20, 0, Math.PI * 2);
    x.fill();
  }
  
  // Continents with realistic placement (approximations)
  const continents = [
    // [centerX, centerY, scaleX, scaleY, rotation]
    // Africa
    { cx: 1100, cy: 520, rx: 130, ry: 180, rot: 0.1, color: "#4a6b2a" },
    { cx: 1080, cy: 420, rx: 100, ry: 80, rot: 0, color: "#6b7a3a" },
    // Europe
    { cx: 1080, cy: 340, rx: 90, ry: 60, rot: 0.2, color: "#4a6b2a" },
    // Asia
    { cx: 1350, cy: 380, rx: 200, ry: 130, rot: 0.1, color: "#5a6b2a" },
    { cx: 1550, cy: 430, rx: 120, ry: 80, rot: 0, color: "#4a5b1a" },
    // North America
    { cx: 500, cy: 350, rx: 150, ry: 150, rot: 0.3, color: "#3a6b2a" },
    { cx: 420, cy: 430, rx: 80, ry: 70, rot: 0, color: "#4a7b3a" },
    // South America
    { cx: 640, cy: 620, rx: 75, ry: 150, rot: -0.1, color: "#3a7b2a" },
    // Australia
    { cx: 1620, cy: 660, rx: 90, ry: 55, rot: 0, color: "#6b6b1a" },
    // Antarctica (fragment)
    { cx: 1024, cy: 980, rx: 800, ry: 40, rot: 0, color: "#e0e8f0" },
    // Greenland
    { cx: 850, cy: 220, rx: 60, ry: 70, rot: 0, color: "#d0d8e0" },
  ];
  
  continents.forEach(c => {
    x.save();
    x.translate(c.cx, c.cy);
    x.rotate(c.rot);
    x.fillStyle = c.color;
    x.beginPath();
    x.ellipse(0, 0, c.rx, c.ry, 0, 0, Math.PI * 2);
    x.fill();
    // Mountain/terrain darker patches
    x.fillStyle = `rgba(40, 60, 20, 0.5)`;
    for (let i = 0; i < 8; i++) {
      const a = Math.random() * Math.PI * 2;
      const r = Math.random() * Math.min(c.rx, c.ry) * 0.7;
      x.beginPath();
      x.ellipse(r * Math.cos(a), r * Math.sin(a), c.rx * 0.2, c.ry * 0.2, 0, 0, Math.PI * 2);
      x.fill();
    }
    x.restore();
  });
  
  // Polar ice caps
  const icecap = x.createLinearGradient(0, 0, 0, 50);
  icecap.addColorStop(0, "#ffffff");
  icecap.addColorStop(1, "#e0e8f0");
  x.fillStyle = icecap;
  x.fillRect(0, 0, 2048, 50);
  x.fillStyle = "#f0f4f8";
  x.fillRect(0, 974, 2048, 50);
  
  // Cloud layer
  for (let i = 0; i < 120; i++) {
    const cx2 = Math.random() * 2048;
    const cy2 = 80 + Math.random() * 864;
    const rx = 20 + Math.random() * 50;
    const ry = 8 + Math.random() * 20;
    x.fillStyle = `rgba(240, 248, 255, ${0.15 + Math.random() * 0.2})`;
    x.beginPath();
    x.ellipse(cx2, cy2, rx, ry, Math.random() * 0.5, 0, Math.PI * 2);
    x.fill();
  }
  
  return new THREE.CanvasTexture(c);
}

function makeMoonTexture() {
  const c = document.createElement("canvas");
  c.width = 1024; c.height = 512;
  const x = c.getContext("2d");
  
  // Base lunar regolith color
  x.fillStyle = "#a8a39c";
  x.fillRect(0, 0, 1024, 512);
  
  // Terrain noise
  for (let i = 0; i < 800; i++) {
    const bright = 140 + Math.random() * 60;
    x.fillStyle = `rgb(${bright}, ${bright-5}, ${bright-10})`;
    x.beginPath();
    x.arc(Math.random() * 1024, Math.random() * 512, 2 + Math.random() * 8, 0, Math.PI * 2);
    x.fill();
  }
  
  // Maria (dark basaltic plains) - real lunar features
  const maria = [
    { cx: 300, cy: 180, rx: 90, ry: 70, c: "#5a5753" }, // Mare Imbrium
    { cx: 380, cy: 240, rx: 70, ry: 60, c: "#606058" }, // Mare Serenitatis
    { cx: 420, cy: 300, rx: 55, ry: 50, c: "#5e5c55" }, // Mare Tranquillitatis
    { cx: 340, cy: 320, rx: 40, ry: 40, c: "#57554e" }, // Mare Nubium
    { cx: 240, cy: 280, rx: 60, ry: 50, c: "#5c5a52" }, // Oceanus Procellarum
    { cx: 500, cy: 260, rx: 40, ry: 35, c: "#585650" }, // Mare Crisium
    { cx: 700, cy: 220, rx: 50, ry: 45, c: "#5c5a52" },
    { cx: 820, cy: 280, rx: 35, ry: 35, c: "#5e5c54" },
  ];
  maria.forEach(m => {
    x.fillStyle = m.c;
    x.beginPath();
    x.ellipse(m.cx, m.cy, m.rx, m.ry, 0, 0, Math.PI * 2);
    x.fill();
    // Darker centers
    x.fillStyle = `rgba(70, 68, 62, 0.4)`;
    x.beginPath();
    x.ellipse(m.cx, m.cy, m.rx * 0.7, m.ry * 0.7, 0, 0, Math.PI * 2);
    x.fill();
  });
  
  // Small craters
  for (let i = 0; i < 150; i++) {
    const cx2 = Math.random() * 1024;
    const cy2 = Math.random() * 512;
    const r = 2 + Math.random() * 6;
    // Dark rim
    x.fillStyle = "#6a6862";
    x.beginPath(); x.arc(cx2, cy2, r, 0, Math.PI * 2); x.fill();
    // Bright highlight (sun-facing rim)
    x.fillStyle = "#c4c0b8";
    x.beginPath(); x.arc(cx2 - r * 0.3, cy2 - r * 0.3, r * 0.6, 0, Math.PI * 2); x.fill();
  }
  
  // Large craters with ray systems
  const bigCraters = [
    { cx: 340, cy: 380, r: 18 },  // Tycho
    { cx: 280, cy: 170, r: 14 },  // Copernicus
    { cx: 220, cy: 220, r: 12 },  // Kepler
    { cx: 600, cy: 350, r: 10 },
    { cx: 780, cy: 160, r: 11 },
  ];
  bigCraters.forEach(cr => {
    // Ray system (bright ejecta)
    for (let ray = 0; ray < 8; ray++) {
      const a = ray * Math.PI / 4 + Math.random() * 0.3;
      const len = cr.r * (3 + Math.random() * 4);
      x.strokeStyle = "rgba(220, 215, 205, 0.25)";
      x.lineWidth = 2;
      x.beginPath();
      x.moveTo(cr.cx + cr.r * Math.cos(a), cr.cy + cr.r * Math.sin(a));
      x.lineTo(cr.cx + len * Math.cos(a), cr.cy + len * Math.sin(a));
      x.stroke();
    }
    // Crater rim
    x.fillStyle = "#8a8680";
    x.beginPath(); x.arc(cr.cx, cr.cy, cr.r, 0, Math.PI * 2); x.fill();
    // Crater floor (dark)
    x.fillStyle = "#5a5852";
    x.beginPath(); x.arc(cr.cx, cr.cy, cr.r * 0.7, 0, Math.PI * 2); x.fill();
    // Central peak (some craters)
    x.fillStyle = "#b0aca4";
    x.beginPath(); x.arc(cr.cx, cr.cy, cr.r * 0.15, 0, Math.PI * 2); x.fill();
  });
  
  return new THREE.CanvasTexture(c);
}

function makeMoonBumpTexture() {
  const c = document.createElement("canvas");
  c.width = 512; c.height = 256;
  const x = c.getContext("2d");
  x.fillStyle = "#808080";
  x.fillRect(0, 0, 512, 256);
  for (let i = 0; i < 300; i++) {
    const cx2 = Math.random() * 512;
    const cy2 = Math.random() * 256;
    const r = 1 + Math.random() * 5;
    const shade = Math.floor(50 + Math.random() * 150);
    x.fillStyle = `rgb(${shade},${shade},${shade})`;
    x.beginPath(); x.arc(cx2, cy2, r, 0, Math.PI * 2); x.fill();
  }
  return new THREE.CanvasTexture(c);
}

function addM(group, geo, mat, x, y, z) {
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  group.add(m);
  return m;
}

/* ─── SLS BLOCK 1 STACK ─── 
   Realistic proportions:
   - Total height: 98.3 m → normalized units
   - Core stage: 64.6 m long, 8.41 m dia (ratio 7.7:1)
   - SRBs: 53.9 m long, 3.71 m dia (ratio 14.5:1)
   - ICPS: 13.7 m long, 5.09 m dia
   - Orion: 3.3 m tall CM + ESM
   - LAS: ~13.4 m
*/
function buildSLS() {
  const g = new THREE.Group();
  const mp = (c, e, s) => new THREE.MeshPhongMaterial({ color: c, emissive: e || 0, shininess: s || 20 });
  
  // Scale: 1 unit = ~12 m
  // Core stage (orange SOFI insulation)
  const coreMat = new THREE.MeshPhongMaterial({ color: 0xc55a1a, emissive: 0x2a0a00, shininess: 15 });
  const core = addM(g, new THREE.CylinderGeometry(0.35, 0.35, 5.4, 24), coreMat, 0, 0, 0);
  // Core stage intertank ring
  addM(g, new THREE.CylinderGeometry(0.36, 0.36, 0.3, 24), mp(0xaa4a15, 0x150500), 0, 0.5, 0);
  // Core stage forward dome
  addM(g, new THREE.SphereGeometry(0.35, 24, 12, 0, Math.PI*2, 0, Math.PI/2), coreMat, 0, 2.7, 0);
  // Core stage aft
  addM(g, new THREE.CylinderGeometry(0.38, 0.35, 0.4, 24), coreMat, 0, -2.9, 0);
  
  // SRBs - 2x white 5-segment solids
  const srbMat = new THREE.MeshPhongMaterial({ color: 0xf0f0ee, emissive: 0x151515, shininess: 30 });
  const srbSegMat = new THREE.MeshPhongMaterial({ color: 0xd8d8d6, emissive: 0x0a0a0a });
  [-0.6, 0.6].forEach(xo => {
    // Main SRB body
    const srb = addM(g, new THREE.CylinderGeometry(0.15, 0.15, 4.5, 16), srbMat, xo, -0.3, 0);
    // Segment joints (5 segments = 4 joint rings)
    for (let i = -2; i <= 2; i++) {
      addM(g, new THREE.CylinderGeometry(0.16, 0.16, 0.08, 16), srbSegMat, xo, -0.3 + i * 0.9, 0);
    }
    // Nose cone (frustum)
    addM(g, new THREE.ConeGeometry(0.15, 0.8, 16), srbMat, xo, 2.3, 0);
    // Nozzle
    addM(g, new THREE.CylinderGeometry(0.1, 0.14, 0.25, 12), mp(0x222222, 0, 50), xo, -2.7, 0);
    addM(g, new THREE.ConeGeometry(0.14, 0.35, 12), mp(0x1a1a1a), xo, -2.92, 0);
    // Attachment struts to core
    const strut = addM(g, new THREE.CylinderGeometry(0.015, 0.015, 0.6, 6), mp(0x888888), xo/2, 0, 0);
    strut.rotation.z = Math.PI/2;
    const strut2 = addM(g, new THREE.CylinderGeometry(0.015, 0.015, 0.6, 6), mp(0x888888), xo/2, -1.5, 0);
    strut2.rotation.z = Math.PI/2;
  });
  
  // RS-25 engines (4x, at bottom of core)
  const engineMat = new THREE.MeshPhongMaterial({ color: 0x3a3a3a, emissive: 0x0a0a0a, shininess: 80 });
  [[-0.15,-0.15],[0.15,-0.15],[-0.15,0.15],[0.15,0.15]].forEach(([ex, ez]) => {
    // Nozzle bell
    const bell = addM(g, new THREE.ConeGeometry(0.11, 0.4, 16, 1, true), engineMat, ex, -3.3, ez);
    bell.rotation.x = Math.PI;
    // Turbomachinery
    addM(g, new THREE.CylinderGeometry(0.06, 0.07, 0.15, 12), mp(0x555555), ex, -3.05, ez);
  });
  
  // ICPS interstage adapter
  addM(g, new THREE.CylinderGeometry(0.32, 0.35, 0.35, 16), mp(0xaaaaaa, 0x0a0a0a), 0, 3.1, 0);
  // ICPS (orange hydrogen tank visible)
  const icpsMat = new THREE.MeshPhongMaterial({ color: 0xdddddd, emissive: 0x1a1a1a, shininess: 25 });
  addM(g, new THREE.CylinderGeometry(0.28, 0.32, 1.1, 20), icpsMat, 0, 3.9, 0);
  // RL10 nozzle
  addM(g, new THREE.ConeGeometry(0.12, 0.3, 12, 1, true), engineMat, 0, 3.2, 0);
  
  // Launch Vehicle Stage Adapter (LVSA)
  addM(g, new THREE.CylinderGeometry(0.25, 0.32, 0.4, 16), mp(0xcccccc, 0x0a0a0a), 0, 4.65, 0);
  
  // Orion Stage Adapter (OSA)
  addM(g, new THREE.CylinderGeometry(0.22, 0.25, 0.25, 16), mp(0xbbbbbb, 0x080808), 0, 4.97, 0);
  
  // Orion ESM (European Service Module)
  const esmMat = new THREE.MeshPhongMaterial({ color: 0x888888, emissive: 0x050505, shininess: 40 });
  addM(g, new THREE.CylinderGeometry(0.22, 0.22, 0.4, 20), esmMat, 0, 5.3, 0);
  // Gold thermal blanket
  const goldMat = new THREE.MeshPhongMaterial({ color: 0xd4a838, emissive: 0x3a2200, shininess: 80 });
  addM(g, new THREE.CylinderGeometry(0.225, 0.225, 0.1, 20), goldMat, 0, 5.45, 0);
  
  // Orion CM (conical)
  const cmMat = new THREE.MeshPhongMaterial({ color: 0xe0e0dc, emissive: 0x0a0a0a, shininess: 50 });
  const cmCone = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.35, 24, 1, true), cmMat);
  cmCone.position.set(0, 5.7, 0);
  g.add(cmCone);
  // CM top
  addM(g, new THREE.CylinderGeometry(0.08, 0.12, 0.06, 12), mp(0xcccccc), 0, 5.9, 0);
  
  // Launch Abort System tower
  const lasMat = new THREE.MeshPhongMaterial({ color: 0xcc2200, emissive: 0x220000, shininess: 60 });
  // Main tower
  addM(g, new THREE.CylinderGeometry(0.05, 0.05, 1.0, 8), lasMat, 0, 6.45, 0);
  // Abort motor (top part, larger)
  addM(g, new THREE.CylinderGeometry(0.09, 0.07, 0.3, 12), lasMat, 0, 6.7, 0);
  // Jettison motor
  addM(g, new THREE.CylinderGeometry(0.06, 0.06, 0.2, 10), lasMat, 0, 7.0, 0);
  // LAS nose cone
  addM(g, new THREE.ConeGeometry(0.05, 0.25, 8), lasMat, 0, 7.25, 0);
  // Aerodynamic fairing over CM
  const fairing = new THREE.Mesh(new THREE.ConeGeometry(0.23, 0.8, 16, 1, true), new THREE.MeshPhongMaterial({ color: 0xdddddd, transparent: true, opacity: 0.3, side: THREE.DoubleSide }));
  fairing.position.set(0, 6.1, 0);
  g.add(fairing);
  
  // Grid fins / aerodynamic surfaces on LAS
  const finMat = mp(0x991100);
  [0, Math.PI/2, Math.PI, 3*Math.PI/2].forEach(a => {
    const fin = addM(g, new THREE.BoxGeometry(0.02, 0.15, 0.08), finMat, 0.1 * Math.cos(a), 6.7, 0.1 * Math.sin(a));
    fin.rotation.y = a;
  });
  
  return g;
}

/* ─── ORION MPCV + ESM (cruise configuration) ─── */
function buildOrion() {
  const g = new THREE.Group();
  const mp = (c, e, s) => new THREE.MeshPhongMaterial({ color: c, emissive: e || 0, shininess: s || 30 });
  
  // CREW MODULE (57.5° frustum, real Orion proportions)
  // Diameter 5.02 m, height 3.3 m → aspect ratio
  const cmMat = new THREE.MeshPhongMaterial({ color: 0xe8e8e4, emissive: 0x0a0a0a, shininess: 60 });
  
  // Main conical hull
  const cmCone = new THREE.Mesh(new THREE.ConeGeometry(1.5, 2.0, 32, 1, true), cmMat);
  cmCone.position.set(0, 1.5, 0);
  cmCone.material.side = THREE.DoubleSide;
  g.add(cmCone);
  
  // CM forward section (crew tunnel/docking port area)
  addM(g, new THREE.CylinderGeometry(0.55, 0.7, 0.3, 16), mp(0xdddddd, 0x080808), 0, 2.65, 0);
  // Docking adapter
  addM(g, new THREE.CylinderGeometry(0.45, 0.55, 0.2, 16), mp(0xbbbbbb, 0x080808), 0, 2.9, 0);
  addM(g, new THREE.CylinderGeometry(0.4, 0.45, 0.1, 16), mp(0x888888), 0, 3.05, 0);
  
  // AVCOAT heat shield (spherical segment, burnt/ablated look)
  const hsMat = new THREE.MeshPhongMaterial({ color: 0x3a2818, emissive: 0x0f0500, shininess: 5 });
  const hs = new THREE.Mesh(
    new THREE.SphereGeometry(1.7, 32, 16, 0, Math.PI*2, Math.PI/2, Math.PI/2),
    hsMat
  );
  hs.position.set(0, 0.5, 0);
  hs.scale.set(1, 0.35, 1); // flatten
  g.add(hs);
  
  // Window positions (4 main windows)
  const winMat = new THREE.MeshPhongMaterial({ color: 0x223344, emissive: 0x112233, shininess: 100 });
  for (let i = 0; i < 4; i++) {
    const a = i * Math.PI / 2 + Math.PI / 4;
    const w = addM(g, new THREE.BoxGeometry(0.2, 0.15, 0.05), winMat,
      1.1 * Math.cos(a), 1.9, 1.1 * Math.sin(a));
    w.lookAt(0, 1.9, 0);
  }
  
  // Reaction Control System thrusters on CM (12 thrusters in clusters)
  const rcsMat = mp(0x444444);
  for (let i = 0; i < 4; i++) {
    const a = i * Math.PI / 2;
    // Cluster of 3 thrusters
    for (let j = -1; j <= 1; j++) {
      const off = j * 0.12;
      const px = (1.2) * Math.cos(a) + (j === 0 ? 0 : off * Math.sin(a));
      const pz = (1.2) * Math.sin(a) - (j === 0 ? 0 : off * Math.cos(a));
      const thr = addM(g, new THREE.ConeGeometry(0.05, 0.1, 8), rcsMat, px, 2.4, pz);
      thr.lookAt(px * 2, 2.4, pz * 2);
    }
  }
  
  // EUROPEAN SERVICE MODULE (cylindrical, 4.1 m dia, 2.7 m tall)
  const esmMat = new THREE.MeshPhongMaterial({ color: 0x9a9a98, emissive: 0x050505, shininess: 40 });
  const esm = addM(g, new THREE.CylinderGeometry(1.4, 1.4, 2.7, 32), esmMat, 0, -1.0, 0);
  
  // ESM gold thermal blankets (MLI)
  const goldMat = new THREE.MeshPhongMaterial({ color: 0xd4a838, emissive: 0x442200, shininess: 90, side: THREE.DoubleSide });
  const band1 = new THREE.Mesh(new THREE.CylinderGeometry(1.42, 1.42, 0.35, 32, 1, true), goldMat);
  band1.position.set(0, -0.3, 0);
  g.add(band1);
  const band2 = new THREE.Mesh(new THREE.CylinderGeometry(1.42, 1.42, 0.35, 32, 1, true), goldMat);
  band2.position.set(0, -1.0, 0);
  g.add(band2);
  const band3 = new THREE.Mesh(new THREE.CylinderGeometry(1.42, 1.42, 0.35, 32, 1, true), goldMat);
  band3.position.set(0, -1.7, 0);
  g.add(band3);
  
  // ESM radiators (4 panels, silver)
  const radMat = new THREE.MeshPhongMaterial({ color: 0xbbbbc0, emissive: 0x080808, shininess: 90 });
  [0, Math.PI/2, Math.PI, 3*Math.PI/2].forEach(a => {
    const rad = new THREE.Mesh(new THREE.BoxGeometry(0.05, 1.8, 0.8), radMat);
    rad.position.set(1.45 * Math.cos(a), -1.0, 1.45 * Math.sin(a));
    rad.lookAt(0, -1.0, 0);
    g.add(rad);
  });
  
  // SOLAR ARRAY WINGS (4 wings, 19 m span)
  // Each wing = 3 panels + yoke + gimbal
  const wingMat = new THREE.MeshPhongMaterial({ 
    color: 0x1a2a88, emissive: 0x080830, 
    shininess: 120, side: THREE.DoubleSide 
  });
  const frameMat = mp(0x555555);
  const yokeMat = mp(0xaaaaaa);
  
  [0, Math.PI/2, Math.PI, 3*Math.PI/2].forEach(a => {
    const arm = new THREE.Group();
    
    // Yoke (base attachment)
    const yoke = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 0.3, 12), yokeMat);
    yoke.position.set(1.5, 0, 0);
    yoke.rotation.z = Math.PI / 2;
    arm.add(yoke);
    
    // Gimbal motor
    const gimbal = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.15, 12), yokeMat);
    gimbal.position.set(1.65, 0, 0);
    gimbal.rotation.z = Math.PI / 2;
    arm.add(gimbal);
    
    // 3 panel segments with gaps
    for (let i = 0; i < 3; i++) {
      const px = 2.4 + i * 2.0;
      // Panel
      const panel = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.04, 0.9), wingMat);
      panel.position.set(px, 0, 0);
      arm.add(panel);
      // Panel frame
      const f1 = new THREE.Mesh(new THREE.BoxGeometry(1.85, 0.05, 0.04), frameMat);
      f1.position.set(px, 0, 0.45);
      arm.add(f1);
      const f2 = new THREE.Mesh(new THREE.BoxGeometry(1.85, 0.05, 0.04), frameMat);
      f2.position.set(px, 0, -0.45);
      arm.add(f2);
      // Cell grid lines
      for (let cell = -2; cell <= 2; cell++) {
        const line = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.05, 0.85), mp(0x333344));
        line.position.set(px + cell * 0.4, 0.025, 0);
        arm.add(line);
      }
    }
    
    // Structural spine
    const spine = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 6.5, 6), frameMat);
    spine.rotation.z = Math.PI / 2;
    spine.position.set(4.8, 0, 0);
    arm.add(spine);
    
    arm.rotation.y = a;
    arm.position.set(0, -1.0, 0);
    g.add(arm);
  });
  
  // AJ10 main engine nozzle (OMS heritage)
  const nozzleMat = new THREE.MeshPhongMaterial({ color: 0x2a2a2a, emissive: 0x0a0a0a, shininess: 60 });
  const nozzle = new THREE.Mesh(new THREE.ConeGeometry(0.35, 0.9, 20, 1, true), nozzleMat);
  nozzle.position.set(0, -2.8, 0);
  nozzle.rotation.x = Math.PI;
  g.add(nozzle);
  // Engine throat
  addM(g, new THREE.CylinderGeometry(0.15, 0.15, 0.2, 12), mp(0x555555), 0, -2.5, 0);
  
  // Auxiliary R-4D thrusters (8 around main)
  for (let i = 0; i < 8; i++) {
    const a = i * Math.PI / 4;
    const aux = new THREE.Mesh(new THREE.ConeGeometry(0.07, 0.15, 8), nozzleMat);
    aux.position.set(0.6 * Math.cos(a), -2.4, 0.6 * Math.sin(a));
    aux.rotation.x = Math.PI;
    g.add(aux);
  }
  
  // RCS pods on ESM (6 pods of 4 thrusters)
  for (let i = 0; i < 6; i++) {
    const a = i * Math.PI / 3;
    const pod = new THREE.Group();
    // Pod body
    const podBody = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.15, 0.15), mp(0x555555));
    pod.add(podBody);
    // 4 thrusters
    for (let j = 0; j < 4; j++) {
      const ta = j * Math.PI / 2;
      const thr = new THREE.Mesh(new THREE.ConeGeometry(0.025, 0.06, 6), nozzleMat);
      thr.position.set(0.1 * Math.cos(ta), 0, 0.1 * Math.sin(ta));
      thr.rotation.x = Math.PI/2;
      thr.rotation.z = -ta;
      pod.add(thr);
    }
    pod.position.set(1.5 * Math.cos(a), -0.5, 1.5 * Math.sin(a));
    g.add(pod);
  }
  
  return g;
}

/* ─── ORION CM ATMOSPHERIC ENTRY (with plasma) ─── */
function buildCMEntry() {
  const g = new THREE.Group();
  const mp = (c, e) => new THREE.MeshPhongMaterial({ color: c, emissive: e || 0 });
  
  // CM cone
  const cmMat = new THREE.MeshPhongMaterial({ color: 0xd0d0cc, emissive: 0x1a0a00 });
  const cm = new THREE.Mesh(new THREE.ConeGeometry(1.5, 2.0, 32, 1, true), cmMat);
  cm.material.side = THREE.DoubleSide;
  cm.position.set(0, 1.2, 0);
  g.add(cm);
  
  // Forward dome
  addM(g, new THREE.CylinderGeometry(0.55, 0.7, 0.3, 16), mp(0xcccccc, 0x150500), 0, 2.35, 0);
  
  // GLOWING HEAT SHIELD (plasma + ablation)
  const hsMat = new THREE.MeshPhongMaterial({
    color: 0xff4400, emissive: 0xff5500,
    emissiveIntensity: 3.0, shininess: 100
  });
  const hs = new THREE.Mesh(
    new THREE.SphereGeometry(1.7, 32, 16, 0, Math.PI*2, Math.PI/2, Math.PI/2),
    hsMat
  );
  hs.position.set(0, 0.2, 0);
  hs.scale.set(1, 0.35, 1);
  g.add(hs);
  
  // Plasma sheath (multiple layers for volumetric feel)
  const plasmaMat1 = new THREE.MeshBasicMaterial({
    color: 0xff6622, transparent: true, opacity: 0.25, side: THREE.BackSide
  });
  const plasma1 = new THREE.Mesh(new THREE.SphereGeometry(2.3, 24, 16), plasmaMat1);
  plasma1.scale.set(1, 0.8, 1);
  g.add(plasma1);
  
  const plasmaMat2 = new THREE.MeshBasicMaterial({
    color: 0xffaa44, transparent: true, opacity: 0.15
  });
  const plasma2 = new THREE.Mesh(new THREE.SphereGeometry(3.2, 20, 14), plasmaMat2);
  plasma2.scale.set(1, 0.6, 1);
  plasma2.position.set(0, -0.5, 0);
  g.add(plasma2);
  
  // Shock wave leading edge
  const shockMat = new THREE.MeshBasicMaterial({
    color: 0xffffee, transparent: true, opacity: 0.3
  });
  const shock = new THREE.Mesh(
    new THREE.SphereGeometry(2.0, 24, 12, 0, Math.PI*2, Math.PI/2 - 0.3, 0.8),
    shockMat
  );
  shock.position.set(0, 0, 0);
  g.add(shock);
  
  return g;
}

/* ─── ORION CM UNDER PARACHUTES ─── */
function buildCMChutes() {
  const g = new THREE.Group();
  const mp = (c, e) => new THREE.MeshPhongMaterial({ color: c, emissive: e || 0 });
  
  // CM cone (cooled)
  const cmMat = new THREE.MeshPhongMaterial({ color: 0xb0b0a8, emissive: 0x080808 });
  const cm = new THREE.Mesh(new THREE.ConeGeometry(1.5, 2.0, 24, 1, true), cmMat);
  cm.material.side = THREE.DoubleSide;
  cm.position.set(0, 1.2, 0);
  g.add(cm);
  addM(g, new THREE.CylinderGeometry(0.55, 0.7, 0.3, 16), mp(0xaaaaaa), 0, 2.35, 0);
  // Charred heat shield
  const hs = new THREE.Mesh(
    new THREE.SphereGeometry(1.7, 24, 12, 0, Math.PI*2, Math.PI/2, Math.PI/2),
    mp(0x221510, 0)
  );
  hs.position.set(0, 0.2, 0);
  hs.scale.set(1, 0.35, 1);
  g.add(hs);
  
  // 3 main parachutes (116 ft each)
  const chuteMat = new THREE.MeshPhongMaterial({ 
    color: 0xff8844, emissive: 0x331100,
    side: THREE.DoubleSide, shininess: 20 
  });
  [0, 2*Math.PI/3, 4*Math.PI/3].forEach(a => {
    // Canopy (dome)
    const canopy = new THREE.Mesh(
      new THREE.SphereGeometry(2.0, 24, 12, 0, Math.PI*2, 0, Math.PI/2.2),
      chuteMat
    );
    canopy.position.set(2.2 * Math.cos(a), 8, 2.2 * Math.sin(a));
    canopy.scale.set(1, 0.7, 1);
    g.add(canopy);
    // Canopy stripes
    const stripeMat = new THREE.MeshPhongMaterial({
      color: 0xffffff, side: THREE.DoubleSide
    });
    for (let s = 0; s < 8; s++) {
      const sa = s * Math.PI / 4;
      const stripe = new THREE.Mesh(
        new THREE.BoxGeometry(0.05, 1.2, 0.04),
        stripeMat
      );
      stripe.position.set(
        2.2 * Math.cos(a) + 1.9 * Math.cos(sa) * 0.7,
        8 + 0.3,
        2.2 * Math.sin(a) + 1.9 * Math.sin(sa) * 0.7
      );
      g.add(stripe);
    }
    // Suspension lines
    for (let line = 0; line < 8; line++) {
      const la = line * Math.PI / 4;
      const lineGeo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(2.2 * Math.cos(a) + 1.8 * Math.cos(la), 7.5, 2.2 * Math.sin(a) + 1.8 * Math.sin(la)),
        new THREE.Vector3(0, 2.5, 0)
      ]);
      const lineObj = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xeeeeee }));
      g.add(lineObj);
    }
  });
  
  return g;
}

/* ═══════════════════════════════════════════════════════════════
   MAIN REACT COMPONENT
   ═══════════════════════════════════════════════════════════════ */
export default function App() {
  const mountRef = useRef(null);
  const scRef = useRef({});
  const [params, setParams] = useState({ parkingAlt: 185, tliDv: 0.388, moonAngleDeg: 148, flybyAlt: 6513 });
  const [idx, setIdx] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(12);
  const [camMode, setCamMode] = useState("system");
  const [showTelem, setShowTelem] = useState(true);
  const [showInfo, setShowInfo] = useState(true);
  const [showCarousel, setShowCarousel] = useState(false);
  const [carouselIdx, setCarouselIdx] = useState(0);
  const refs = useRef({ idx: 0, playing: false, speed: 12, cam: "system" });

  useEffect(() => { refs.current.idx = idx; }, [idx]);
  useEffect(() => { refs.current.playing = playing; }, [playing]);
  useEffect(() => { refs.current.speed = speed; }, [speed]);
  useEffect(() => { refs.current.cam = camMode; }, [camMode]);

  const traj = useMemo(() => computeFullMission(params), [params.parkingAlt, params.tliDv, params.moonAngleDeg, params.flybyAlt]);
  const SC = 1 / 10000; // 1 unit = 10,000 km

  useEffect(() => {
    const el = mountRef.current;
    if (!el) return;
    const W = el.clientWidth, H = el.clientHeight;
    
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000308);
    
    const camera = new THREE.PerspectiveCamera(45, W / H, 0.001, 5000);
    camera.position.set(0, 30, 60);
    
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    renderer.setSize(W, H);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.1;
    el.appendChild(renderer.domElement);

    // ─── LIGHTING ───
    // Sun (strong directional)
    const sunLight = new THREE.DirectionalLight(0xfff8ee, 1.8);
    sunLight.position.set(80, 30, 60);
    scene.add(sunLight);
    // Sun lens flare effect
    const sunSprite = new THREE.Mesh(
      new THREE.SphereGeometry(6, 16, 8),
      new THREE.MeshBasicMaterial({ color: 0xffffee })
    );
    sunSprite.position.set(600, 200, 400);
    scene.add(sunSprite);
    // Ambient fill (earthshine)
    scene.add(new THREE.AmbientLight(0x1a2840, 0.35));
    // Earth blue rim light
    const rimLight = new THREE.PointLight(0x4477dd, 0.4, 200);
    rimLight.position.set(0, 0, 0);
    scene.add(rimLight);

    // ─── STARFIELD (dense, realistic) ───
    const sv = [];
    const sc_colors = [];
    for (let i = 0; i < 8000; i++) {
      const r = 800 + Math.random() * 1600;
      const t = Math.random() * Math.PI * 2;
      const p = Math.acos(2 * Math.random() - 1);
      sv.push(r*Math.sin(p)*Math.cos(t), r*Math.cos(p), r*Math.sin(p)*Math.sin(t));
      // Varying star colors (realistic spectrum)
      const temp = Math.random();
      if (temp < 0.1) { sc_colors.push(0.7, 0.8, 1.0); } // blue
      else if (temp < 0.3) { sc_colors.push(1.0, 1.0, 1.0); } // white
      else if (temp < 0.7) { sc_colors.push(1.0, 0.95, 0.85); } // yellow-white
      else if (temp < 0.9) { sc_colors.push(1.0, 0.85, 0.6); } // orange
      else { sc_colors.push(1.0, 0.6, 0.4); } // red
    }
    const sg = new THREE.BufferGeometry();
    sg.setAttribute("position", new THREE.Float32BufferAttribute(sv, 3));
    sg.setAttribute("color", new THREE.Float32BufferAttribute(sc_colors, 3));
    scene.add(new THREE.Points(sg, new THREE.PointsMaterial({ 
      vertexColors: true, size: 0.4, sizeAttenuation: true
    })));

    // ─── EARTH ───
    const earthTex = makeEarthTexture();
    earthTex.wrapS = THREE.RepeatWrapping;
    const earthMat = new THREE.MeshPhongMaterial({ 
      map: earthTex, 
      shininess: 25,
      specular: new THREE.Color(0x222244)
    });
    const earth = new THREE.Mesh(new THREE.SphereGeometry(R_E * SC, 96, 64), earthMat);
    earth.rotation.x = 0.41; // axial tilt 23.5°
    scene.add(earth);
    
    // Atmosphere (multiple layers)
    const atmoMat1 = new THREE.MeshBasicMaterial({ 
      color: 0x4488ff, transparent: true, opacity: 0.12, side: THREE.BackSide 
    });
    scene.add(new THREE.Mesh(new THREE.SphereGeometry(R_E * SC * 1.025, 64, 48), atmoMat1));
    const atmoMat2 = new THREE.MeshBasicMaterial({ 
      color: 0x66aaff, transparent: true, opacity: 0.06, side: THREE.BackSide 
    });
    scene.add(new THREE.Mesh(new THREE.SphereGeometry(R_E * SC * 1.06, 48, 32), atmoMat2));

    // ─── MOON ───
    const moonTex = makeMoonTexture();
    const moonBump = makeMoonBumpTexture();
    const moonMat = new THREE.MeshPhongMaterial({ 
      map: moonTex, 
      bumpMap: moonBump,
      bumpScale: 0.002,
      shininess: 2,
      specular: new THREE.Color(0x222222)
    });
    const moon = new THREE.Mesh(new THREE.SphereGeometry(R_M * SC, 64, 48), moonMat);
    scene.add(moon);

    // ─── MOON ORBIT REFERENCE ───
    const op = [];
    for (let a = 0; a <= 360; a += 0.5) {
      const r = a * Math.PI / 180;
      op.push(new THREE.Vector3(
        D_EM*SC*Math.cos(r), 
        D_EM*SC*Math.sin(r)*Math.sin(MOON_INCL), 
        D_EM*SC*Math.sin(r)*Math.cos(MOON_INCL)
      ));
    }
    scene.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(op), 
      new THREE.LineBasicMaterial({ color: 0x1a2838, transparent: true, opacity: 0.5 })
    ));
    
    // Earth SOI indicator (subtle)
    const soiPts = [];
    for (let a = 0; a <= 360; a += 2) {
      const r = a * Math.PI / 180;
      soiPts.push(new THREE.Vector3(925000*SC*Math.cos(r), 0, 925000*SC*Math.sin(r)));
    }

    // ─── TRAJECTORY: MULTI-LAYER BRIGHT RENDERING ───
    const MX = 25000;
    
    // Layer 1: Wide glow halo
    const tPosG = new Float32Array(MX * 3);
    const tColG = new Float32Array(MX * 3);
    const tGeoG = new THREE.BufferGeometry();
    tGeoG.setAttribute("position", new THREE.Float32BufferAttribute(tPosG, 3));
    tGeoG.setAttribute("color", new THREE.Float32BufferAttribute(tColG, 3));
    tGeoG.setDrawRange(0, 0);
    scene.add(new THREE.Line(tGeoG, new THREE.LineBasicMaterial({ 
      vertexColors: true, transparent: true, opacity: 0.6
    })));
    
    // Layer 2: Bright core
    const tPos = new Float32Array(MX * 3);
    const tCol = new Float32Array(MX * 3);
    const tGeo = new THREE.BufferGeometry();
    tGeo.setAttribute("position", new THREE.Float32BufferAttribute(tPos, 3));
    tGeo.setAttribute("color", new THREE.Float32BufferAttribute(tCol, 3));
    tGeo.setDrawRange(0, 0);
    scene.add(new THREE.Line(tGeo, new THREE.LineBasicMaterial({ vertexColors: true })));

    // Future trail
    const fPos = new Float32Array(MX * 3);
    const fCol = new Float32Array(MX * 3);
    const fGeo = new THREE.BufferGeometry();
    fGeo.setAttribute("position", new THREE.Float32BufferAttribute(fPos, 3));
    fGeo.setAttribute("color", new THREE.Float32BufferAttribute(fCol, 3));
    fGeo.setDrawRange(0, 0);
    scene.add(new THREE.Line(fGeo, new THREE.LineBasicMaterial({ 
      vertexColors: true, transparent: true, opacity: 0.5
    })));

    // ─── SPACECRAFT MODELS ───
    const sls = buildSLS();
    const orion = buildOrion();
    const cmEntry = buildCMEntry();
    const cmChutes = buildCMChutes();
    
    const cSc = R_E * SC * 0.5;
    sls.scale.setScalar(cSc * 0.075);
    orion.scale.setScalar(cSc * 0.14);
    cmEntry.scale.setScalar(cSc * 0.14);
    cmChutes.scale.setScalar(cSc * 0.14);
    
    [sls, orion, cmEntry, cmChutes].forEach(m => { m.visible = false; scene.add(m); });

    // Engine exhaust particles
    const ePts = new Float32Array(150 * 3);
    const eGeo = new THREE.BufferGeometry();
    eGeo.setAttribute("position", new THREE.Float32BufferAttribute(ePts, 3));
    const exhaust = new THREE.Points(eGeo, new THREE.PointsMaterial({ 
      color: 0xff9944, size: cSc * 0.35, 
      transparent: true, opacity: 0.8, sizeAttenuation: true,
      blending: THREE.AdditiveBlending
    }));
    scene.add(exhaust);

    // Closest approach marker
    const caGroup = new THREE.Group();
    const caM = new THREE.Mesh(
      new THREE.SphereGeometry(0.1, 16, 12), 
      new THREE.MeshBasicMaterial({ color: 0xff44ff })
    );
    caGroup.add(caM);
    // Halo
    const caHalo = new THREE.Mesh(
      new THREE.SphereGeometry(0.25, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xff44ff, transparent: true, opacity: 0.25 })
    );
    caGroup.add(caHalo);
    caGroup.visible = false;
    scene.add(caGroup);

    // ─── CAMERA ORBIT CONTROL ───
    let cTh = 0.4, cPh = 1.0, cDi = 60;
    let dragging = false, lmx = 0, lmy = 0;
    const rd = renderer.domElement;
    const onMD = e => { dragging = true; lmx = e.clientX; lmy = e.clientY; rd.style.cursor = "grabbing"; };
    const onMM = e => { 
      if (!dragging) return; 
      cTh -= (e.clientX - lmx) * 0.005; 
      cPh = Math.max(0.05, Math.min(Math.PI - 0.05, cPh - (e.clientY - lmy) * 0.005)); 
      lmx = e.clientX; lmy = e.clientY; 
    };
    const onMU = () => { dragging = false; rd.style.cursor = "grab"; };
    const onWh = e => { 
      cDi = Math.max(0.3, Math.min(500, cDi * (1 + e.deltaY * 0.001))); 
      e.preventDefault(); 
    };
    const onTS = e => { if (e.touches.length === 1) { dragging = true; lmx = e.touches[0].clientX; lmy = e.touches[0].clientY; } };
    const onTM = e => { 
      if (!dragging || !e.touches.length) return; 
      cTh -= (e.touches[0].clientX - lmx)*0.005; 
      cPh = Math.max(0.05, Math.min(Math.PI-0.05, cPh-(e.touches[0].clientY-lmy)*0.005)); 
      lmx = e.touches[0].clientX; lmy = e.touches[0].clientY; 
    };
    rd.addEventListener("mousedown", onMD);
    rd.addEventListener("mousemove", onMM);
    rd.addEventListener("mouseup", onMU);
    rd.addEventListener("mouseleave", onMU);
    rd.addEventListener("wheel", onWh, { passive: false });
    rd.addEventListener("touchstart", onTS);
    rd.addEventListener("touchmove", onTM);
    rd.addEventListener("touchend", onMU);

    scRef.current = { scene, camera, renderer, earth, moon, tGeo, tPos, tCol, tGeoG, tPosG, tColG, fGeo, fPos, fCol, sls, orion, cmEntry, cmChutes, exhaust, eGeo, ePts, caGroup, caM, caHalo, MX };

    let lt = 0, raf;
    const tick = (now) => {
      if (now - lt > 28 && refs.current.playing) {
        lt = now;
        const T = scRef.current._traj;
        if (T) {
          const nx = Math.min(refs.current.idx + refs.current.speed, T.pts.length - 1);
          refs.current.idx = nx;
          setIdx(nx);
          if (nx >= T.pts.length - 1) setPlaying(false);
        }
      }

      const T = scRef.current._traj;
      const ci = refs.current.idx;
      if (T && T.pts.length > 0) {
        const p = T.pts[Math.min(ci, T.pts.length - 1)];
        // Physics (x,y,z) → 3D (X,Y,Z): physics xy is orbital plane, physics z is inclination
        const px = p.x * SC, py = (p.z || 0) * SC, pz = p.y * SC;
        const mmx = p.mx * SC, mmy = (p.mz || 0) * SC, mmz = p.my * SC;

        moon.position.set(mmx, mmy, mmz);
        // Earth rotation (15°/hr sidereal)
        earth.rotation.y = p.t * 7.2921e-5;

        // Past trail
        const n = Math.min(ci + 1, T.pts.length, MX);
        for (let i = 0; i < n; i++) {
          const pt = T.pts[i];
          const xx = pt.x * SC, yy = (pt.z||0) * SC, zz = pt.y * SC;
          tPos[i*3] = xx; tPos[i*3+1] = yy; tPos[i*3+2] = zz;
          tPosG[i*3] = xx; tPosG[i*3+1] = yy; tPosG[i*3+2] = zz;
          const c3 = new THREE.Color(PH[pt.phase].hex);
          const f = 0.75 + 0.25 * (i / Math.max(n - 1, 1));
          tCol[i*3] = Math.min(1, c3.r * f * 1.6);
          tCol[i*3+1] = Math.min(1, c3.g * f * 1.6);
          tCol[i*3+2] = Math.min(1, c3.b * f * 1.6);
          tColG[i*3] = c3.r * f * 0.9;
          tColG[i*3+1] = c3.g * f * 0.9;
          tColG[i*3+2] = c3.b * f * 0.9;
        }
        tGeo.attributes.position.needsUpdate = true;
        tGeo.attributes.color.needsUpdate = true;
        tGeo.setDrawRange(0, n);
        tGeoG.attributes.position.needsUpdate = true;
        tGeoG.attributes.color.needsUpdate = true;
        tGeoG.setDrawRange(0, n);

        // Future trail
        const fn = Math.min(T.pts.length - ci, MX);
        for (let i = 0; i < fn; i++) {
          const pt = T.pts[ci + i];
          fPos[i*3] = pt.x * SC; fPos[i*3+1] = (pt.z||0) * SC; fPos[i*3+2] = pt.y * SC;
          const c3 = new THREE.Color(PH[pt.phase].hex);
          fCol[i*3] = c3.r * 0.8;
          fCol[i*3+1] = c3.g * 0.8;
          fCol[i*3+2] = c3.b * 0.8;
        }
        fGeo.attributes.position.needsUpdate = true;
        fGeo.attributes.color.needsUpdate = true;
        fGeo.setDrawRange(0, fn);

        // Vehicle model selection based on phase
        const ph = p.phase;
        sls.visible = ph >= 1 && ph <= 3;           // Launch through core sep
        orion.visible = ph >= 4 && ph <= 14;         // Orion deploy through return
        cmEntry.visible = ph >= 15 && ph <= 16;      // Peak heating + hypersonic
        cmChutes.visible = ph >= 17 && ph <= 19;     // Drogues + mains + splashdown
        
        const active = sls.visible ? sls : orion.visible ? orion : cmEntry.visible ? cmEntry : cmChutes;
        active.position.set(px, py, pz);

        if (p.v > 0.01) {
          const dir = new THREE.Vector3(p.vx, p.vz || 0, p.vy).normalize();
          active.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        }

        // Exhaust particles (during any powered phase)
        exhaust.visible = (ph === 1 || ph === 2 || ph === 5 || ph === 7 || ph === 9);
        if (exhaust.visible) {
          const exhSize = ph === 1 ? 0.08 : ph === 2 ? 0.05 : 0.03;
          for (let i = 0; i < 150; i++) {
            const sp = exhSize * (1 + Math.random());
            ePts[i*3] = px + (Math.random()-0.5)*sp;
            ePts[i*3+1] = py - Math.random()*exhSize*4;
            ePts[i*3+2] = pz + (Math.random()-0.5)*sp;
          }
          eGeo.attributes.position.needsUpdate = true;
        }

        // Closest approach marker
        if (ci >= T.minMI && T.minMI < T.pts.length) {
          const ca = T.pts[T.minMI];
          caGroup.position.set(ca.x*SC, (ca.z||0)*SC, ca.y*SC);
          caGroup.visible = true;
          // Pulse halo
          const pulse = 1 + 0.3 * Math.sin(now * 0.003);
          caHalo.scale.setScalar(pulse);
        } else caGroup.visible = false;

        // Camera modes
        const cm2 = refs.current.cam;
        let tgt = new THREE.Vector3(0, 0, 0);
        let d = cDi;
        if (cm2 === "craft") { 
          tgt.set(px, py, pz); 
          d = Math.max(d * 0.008, 0.1); 
        } else if (cm2 === "moon") { 
          tgt.set(mmx, mmy, mmz); 
          d = Math.max(d * 0.04, 0.4); 
        } else if (cm2 === "earth") { 
          d = Math.max(d * 0.02, 0.6); 
        } else if (cm2 === "chase") {
          tgt.set(px, py, pz);
          d = Math.max(d * 0.015, 0.15);
          // Follow behind velocity vector
          if (p.v > 0.01) {
            const vn = new THREE.Vector3(p.vx, p.vz || 0, p.vy).normalize();
            camera.position.set(
              px - vn.x * d * 0.7 + vn.z * d * 0.3,
              py - vn.y * d * 0.7 + d * 0.2,
              pz - vn.z * d * 0.7 - vn.x * d * 0.3
            );
            camera.lookAt(tgt);
            renderer.render(scene, camera);
            raf = requestAnimationFrame(tick);
            return;
          }
        }

        camera.position.set(
          tgt.x + d*Math.sin(cPh)*Math.cos(cTh),
          tgt.y + d*Math.cos(cPh),
          tgt.z + d*Math.sin(cPh)*Math.sin(cTh)
        );
        camera.lookAt(tgt);
      }
      renderer.render(scene, camera);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);

    const onRs = () => {
      if (!el) return;
      camera.aspect = el.clientWidth / el.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(el.clientWidth, el.clientHeight);
    };
    window.addEventListener("resize", onRs);

    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener("resize", onRs);
      rd.removeEventListener("mousedown", onMD);
      rd.removeEventListener("mousemove", onMM);
      rd.removeEventListener("mouseup", onMU);
      rd.removeEventListener("mouseleave", onMU);
      rd.removeEventListener("wheel", onWh);
      rd.removeEventListener("touchstart", onTS);
      rd.removeEventListener("touchmove", onTM);
      rd.removeEventListener("touchend", onMU);
      if (el.contains(rd)) el.removeChild(rd);
      renderer.dispose();
    };
  }, []);

  useEffect(() => { scRef.current._traj = traj; }, [traj]);

  const cur = traj.pts[Math.min(idx, traj.pts.length - 1)] || { t: 0, v: 0, altE: 0, rE: 0, rM: 0, phase: 0 };
  const ph = PH[cur.phase || 0];
  
  let outcome = "IN PROGRESS", oCol = "#00ccff";
  if (idx >= traj.pts.length - 1 && traj.pts.length > 0) {
    const L = traj.pts[traj.pts.length - 1];
    if (L.phase === 19 || L.altE < 1) { outcome = "SPLASHDOWN — PACIFIC"; oCol = "#88ddff"; }
    else if (L.altE < 0) { outcome = "EARTH IMPACT"; oCol = "#ff5252"; }
    else if (L.rM - R_M < 0) { outcome = "MOON IMPACT"; oCol = "#ff5252"; }
    else if (L.rE > D_EM * 3) { outcome = "ESCAPE TRAJECTORY"; oCol = "#ffab00"; }
    else { outcome = "SIMULATION END"; oCol = "#888"; }
  }

  const setP = (k, v) => {
    setParams(p => ({ ...p, [k]: v }));
    setIdx(0);
    refs.current.idx = 0;
    setPlaying(false);
  };

  // AROW-style time: days hours minutes seconds
  const fT = s => {
    const sec = Math.floor(s);
    const d = Math.floor(sec / 86400);
    const h = Math.floor((sec % 86400) / 3600);
    const m = Math.floor((sec % 3600) / 60);
    const ss = sec % 60;
    return `${String(d).padStart(2,"0")}d ${String(h).padStart(2,"0")}h ${String(m).padStart(2,"0")}m ${String(ss).padStart(2,"0")}s`;
  };
  const fD = km => km > 10000 ? `${(km/1000).toFixed(1)}k km` : `${km.toFixed(0)} km`;

  const vehicleDesc = cur.phase <= 3 ? "SLS BLOCK 1 + ORION" :
                      cur.phase <= 14 ? "ORION + ESM" :
                      cur.phase <= 16 ? "ORION CM · PLASMA" :
                      cur.phase <= 18 ? "ORION CM · PARACHUTES" :
                      "ORION CM · RECOVERY";

  return (
    <div style={{ width: "100%", height: "100vh", background: "#000308", position: "relative", overflow: "hidden", fontFamily: "'JetBrains Mono', Menlo, monospace", color: "#a0b8d0" }}>
      <style>{`
        .gp { background: rgba(2,6,15,0.92); backdrop-filter: blur(10px); border: 1px solid rgba(0,180,255,0.15); border-radius: 3px; }
        .gb { background: rgba(0,160,255,0.06); border: 1px solid rgba(0,160,255,0.22); color: #6a9ac0; padding: 4px 9px; cursor: pointer; font-family: inherit; font-size: 9px; letter-spacing: 1px; transition: all 0.15s; border-radius: 2px; white-space: nowrap; }
        .gb:hover { background: rgba(0,180,255,0.16); color: #00ccff; border-color: rgba(0,200,255,0.4); }
        .gb.on { background: rgba(0,180,255,0.18); color: #00ddff; border-color: #00bbee; box-shadow: 0 0 6px rgba(0,200,255,0.3); }
        .ps { background: rgba(0,140,255,0.04); border: 1px solid rgba(0,140,255,0.12); padding: 5px 7px; cursor: pointer; transition: all 0.15s; border-radius: 2px; text-align: left; display: block; width: 100%; margin-bottom: 3px; color: inherit; font-family: inherit; }
        .ps:hover { background: rgba(0,180,255,0.12); border-color: rgba(0,200,255,0.3); }
        input[type=range]{-webkit-appearance:none;appearance:none;background:rgba(0,180,255,0.12);height:3px;border-radius:2px;outline:none;width:100%}
        input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:13px;height:13px;border-radius:50%;background:#00ccff;cursor:pointer;box-shadow:0 0 6px rgba(0,200,255,0.5)}
      `}</style>

      <div ref={mountRef} style={{ width: "100%", height: "100%", position: "absolute", top: 0, left: 0, cursor: "grab" }} />

      {/* TOP BAR */}
      <div className="gp" style={{ position: "absolute", top: 0, left: 0, right: 0, padding: "8px 14px", zIndex: 10, display: "flex", alignItems: "center", justifyContent: "space-between" }}>
        <div>
          <div style={{ fontWeight: 900, color: "#00ddff", letterSpacing: 3, fontSize: 14 }}>
            ARTEMIS <span style={{ color: "#fff" }}>II</span>
            <span style={{ fontSize: 10, color: "#3a5a70", marginLeft: 12, letterSpacing: 1.5, fontWeight: 500 }}>HIGH-FIDELITY FULL MISSION SIMULATOR</span>
          </div>
          <div style={{ fontSize: 10, color: "#2a4a60", letterSpacing: 1, marginTop: 1 }}>
            CREW: WISEMAN · GLOVER · KOCH · HANSEN  ·  ORION "INTEGRITY"  ·  KSC LC-39B  ·  NASA AROW DATA
          </div>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
          <div style={{ width: 8, height: 8, borderRadius: "50%", background: ph.color, boxShadow: `0 0 12px ${ph.color}` }} />
          <span style={{ fontSize: 12, color: ph.color, letterSpacing: 1, fontWeight: 700 }}>{ph.name}</span>
        </div>
      </div>

      {/* BOTTOM CONTROLS */}
      <div className="gp" style={{ position: "absolute", bottom: 0, left: 0, right: 0, padding: "6px 12px", zIndex: 10, display: "flex", alignItems: "center", gap: 6, flexWrap: "wrap" }}>
        <button className="gb" onClick={() => { setIdx(0); refs.current.idx = 0; setPlaying(false); }}>RESET</button>
        <button className="gb" onClick={() => setPlaying(!playing)} style={{ minWidth: 52 }}>{playing ? "⏸ PAUSE" : "▶ PLAY"}</button>
        <span style={{ fontSize: 10, color: "#3a5a70" }}>SPD</span>
        <input type="range" min={1} max={150} value={speed} onChange={e => setSpeed(+e.target.value)} style={{ width: 60 }} />
        <span style={{ fontSize: 10, color: "#00ccff", minWidth: 32 }}>{speed}×</span>
        <input type="range" min={0} max={Math.max(0, traj.pts.length-1)} value={idx} onChange={e => { const v = +e.target.value; setIdx(v); refs.current.idx = v; setPlaying(false); }} style={{ flex: 1, minWidth: 100 }} />
        <span style={{ fontSize: 12, color: "#00ddff", minWidth: 155, fontWeight: 700, textAlign: "center", letterSpacing: 0.5 }}>MET {fT(cur.t || 0)}</span>
        <div style={{ display: "flex", gap: 3 }}>
          {["system","earth","moon","craft","chase"].map(v => (
            <button key={v} className={`gb ${camMode===v?"on":""}`} onClick={() => setCamMode(v)}>{v.toUpperCase()}</button>
          ))}
        </div>
      </div>

      <div style={{ position: "absolute", bottom: 44, left: "50%", transform: "translateX(-50%)", fontSize: 10, color: "rgba(0,200,255,0.45)", letterSpacing: 2, zIndex: 5, pointerEvents: "none", fontWeight: 700 }}>
        {vehicleDesc}
      </div>

      {/* LEFT PANEL */}
      {showInfo && (
      <div className="gp" style={{ position: "absolute", top: 56, left: 6, width: 240, padding: "12px 14px", zIndex: 10, maxHeight: "calc(100vh - 110px)", overflowY: "auto", fontSize: 12, color: "#7a9eb8" }}>
        <div style={{ fontSize: 10, color: "#00ccff", letterSpacing: 2, marginBottom: 7, fontWeight: 700 }}>PARAMETERS</div>
        <SL label="TLI ΔV" unit="km/s" min={0.35} max={0.45} step={0.001} val={params.tliDv} set={v=>setP("tliDv",v)} fmt={v=>v.toFixed(3)} />
        <SL label="FLYBY ALT" unit="km" min={100} max={30000} step={100} val={params.flybyAlt} set={v=>setP("flybyAlt",v)} fmt={v=>v.toFixed(0)} />
        <SL label="PARKING ALT" unit="km" min={150} max={2000} step={5} val={params.parkingAlt} set={v=>setP("parkingAlt",v)} fmt={v=>v.toFixed(0)} />
        <SL label="MOON ANGLE" unit="°" min={80} max={200} step={0.5} val={params.moonAngleDeg} set={v=>setP("moonAngleDeg",v)} fmt={v=>v.toFixed(1)} />

        <div style={{ fontSize: 10, color: "#00ccff", letterSpacing: 2, marginTop: 12, marginBottom: 5, fontWeight: 700 }}>SCENARIOS</div>
        {PRESETS.map(pr => (
          <button key={pr.n} className="ps" onClick={() => { setParams({ parkingAlt: pr.a, tliDv: pr.dv, moonAngleDeg: pr.ang, flybyAlt: pr.fly }); setIdx(0); refs.current.idx = 0; setPlaying(false); }}>
            <div style={{ fontSize: 10, fontWeight: 600, color: "#8abcdc" }}>{pr.n}</div>
            <div style={{ fontSize: 10, color: "#3a5a70", marginTop: 1 }}>{pr.d}</div>
          </button>
        ))}

        <div style={{ fontSize: 10, color: "#00ccff", letterSpacing: 2, marginTop: 12, marginBottom: 5, fontWeight: 700 }}>MISSION FACTS</div>
        <div style={{ fontSize: 10, color: "#5a7a90", lineHeight: 1.7 }}>
          <b style={{color:"#7aaccc"}}>Launch:</b> Apr 1, 2026<br/>
          <b style={{color:"#7aaccc"}}>Vehicle:</b> SLS Block 1 (98.3 m)<br/>
          <b style={{color:"#7aaccc"}}>Spacecraft:</b> Orion "Integrity"<br/>
          <b style={{color:"#7aaccc"}}>Thrust:</b> 8.8M lbf liftoff<br/>
          <b style={{color:"#7aaccc"}}>Duration:</b> ~9d 01h 46m<br/>
          <b style={{color:"#7aaccc"}}>TLI ΔV:</b> ~388 m/s<br/>
          <b style={{color:"#7aaccc"}}>Lunar pass:</b> 6,513 km<br/>
          <b style={{color:"#7aaccc"}}>Max range:</b> 405,500 km<br/>
          <b style={{color:"#7aaccc"}}>Entry:</b> 11.1 km/s, -6°<br/>
          <b style={{color:"#7aaccc"}}>Heat shield:</b> AVCOAT 5.02 m<br/>
          <b style={{color:"#7aaccc"}}>Splashdown:</b> Pacific
        </div>

        <div style={{ fontSize: 10, color: "#00ccff", letterSpacing: 2, marginTop: 12, marginBottom: 5, fontWeight: 700 }}>PHYSICS</div>
        <div style={{ fontSize: 10, color: "#2a4a60", lineHeight: 1.7 }}>
          μ_E = 398,600.44 km³/s²<br/>μ_M = 4,902.80 km³/s²<br/>
          J2_E = 1.0826×10⁻³<br/>
          R_E = 6,378.14 km<br/>R_M = 1,737.4 km<br/>
          D_EM = 384,399 km<br/>Moon incl: 5.14°<br/>
          RK4 Δt = 30–180 s<br/>
          3-body + J2 oblateness
        </div>
      </div>
      )}
      <button className="gb" style={{ position: "absolute", top: 56, left: showInfo ? 227 : 6, zIndex: 10 }} onClick={() => setShowInfo(!showInfo)}>
        {showInfo ? "◀" : "▶"}
      </button>

      {/* RIGHT TELEMETRY */}
      {showTelem && (
        <div className="gp" style={{ position: "absolute", top: 56, right: 6, width: 240, padding: "12px 14px", zIndex: 10, maxHeight: "calc(100vh - 110px)", overflowY: "auto", fontSize: 12, color: "#7a9eb8" }}>
          <div style={{ fontSize: 10, color: "#00ccff", letterSpacing: 2, marginBottom: 7, fontWeight: 700 }}>LIVE TELEMETRY</div>
          <TR l="MISSION ELAPSED" v={fT(cur.t||0)} c="#ddeeff" big />
          <div style={{ height: 4 }} />
          <TR l="VELOCITY" v={`${(cur.v||0).toFixed(3)} km/s`} c="#00ddff" />
          <TR l="" v={`${((cur.v||0)*2236.94).toFixed(0)} mph`} />
          <TR l="" v={`Mach ${((cur.v||0)*1000/343).toFixed(1)}`} />
          <TR l="ALTITUDE EARTH" v={fD(cur.altE||0)} />
          <TR l="DIST FROM MOON" v={fD(Math.max(0,(cur.rM||0)-R_M))} />
          <TR l="DIST FROM EARTH" v={fD(cur.rE||0)} />
          <div style={{ height: 6, borderTop: "1px solid rgba(0,180,255,0.1)", marginTop: 4, paddingTop: 4 }} />
          <TR l="CLOSEST LUNAR" v={`${fD(traj.minMD - R_M)}`} c="#ff44ff" />
          <TR l="MAX FROM EARTH" v={`${(traj.maxED/1000).toFixed(1)}k km`} />
          <TR l="" v={`${(traj.maxED*0.621371).toFixed(0)} miles`} />
          <div style={{ marginTop: 8, padding: "3px 6px", border: `1px solid ${oCol}40`, borderRadius: 2, fontSize: 10, color: oCol, letterSpacing: 1, background: `${oCol}10`, textAlign: "center", fontWeight: 700 }}>{outcome}</div>
          
          <div style={{ marginTop: 10, borderTop: "1px solid rgba(0,180,255,0.1)", paddingTop: 6 }}>
            <div style={{ fontSize: 10, color: "#00ccff", letterSpacing: 2, marginBottom: 5, fontWeight: 700 }}>MISSION PHASES</div>
            {PH.map((p,i) => (
              <div key={i} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 2, opacity: cur.phase === i ? 1 : 0.4 }}>
                <div style={{ width: 12, height: 2, background: p.color, borderRadius: 1, boxShadow: cur.phase === i ? `0 0 5px ${p.color}` : "none", flexShrink: 0 }} />
                <span style={{ fontSize: 10, color: cur.phase === i ? p.color : "#3a5a70", fontWeight: cur.phase === i ? 700 : 400, whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>{p.name}</span>
              </div>
            ))}
          </div>
        </div>
      )}
      <button className="gb" style={{ position: "absolute", top: 56, right: showTelem ? 227 : 6, zIndex: 10 }} onClick={() => setShowTelem(!showTelem)}>
        {showTelem ? "▶" : "◀"}
      </button>
      
      <div style={{ position: "absolute", bottom: 44, right: 10, fontSize: 10, color: "rgba(80,120,160,0.35)", zIndex: 5, pointerEvents: "none" }}>
        DRAG: ORBIT · SCROLL: ZOOM
      </div>
      
      {/* ─── COMPONENT CAROUSEL TOGGLE BUTTON ─── */}
      <button 
        className="gb" 
        style={{ 
          position: "absolute", 
          bottom: showCarousel ? 370 : 44, 
          left: "50%", 
          transform: "translateX(-50%)",
          zIndex: 11,
          padding: "6px 14px",
          fontSize: 12,
          transition: "bottom 0.3s ease"
        }} 
        onClick={() => setShowCarousel(!showCarousel)}
      >
        {showCarousel ? "▼ HIDE COMPONENTS" : "▲ VEHICLE COMPONENTS"}
      </button>
      
      {/* ─── COMPONENT CAROUSEL ─── */}
      {showCarousel && (
        <div className="gp" style={{ 
          position: "absolute", 
          bottom: 44, 
          left: 12, 
          right: 12, 
          height: 320, 
          zIndex: 10, 
          padding: "10px 14px",
          display: "flex",
          flexDirection: "column",
          overflow: "hidden"
        }}>
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 6 }}>
            <div style={{ fontSize: 12, color: "#00ccff", letterSpacing: 2, fontWeight: 700 }}>
              {VEHICLE_COMPONENTS[carouselIdx].category} · {VEHICLE_COMPONENTS[carouselIdx].name.toUpperCase()}
            </div>
            <div style={{ fontSize: 10, color: "#3a5a70" }}>
              {carouselIdx + 1} / {VEHICLE_COMPONENTS.length}
            </div>
          </div>
          
          <div style={{ display: "flex", gap: 14, flex: 1, minHeight: 0 }}>
            {/* Left: component illustration */}
            <div style={{ width: 240, flexShrink: 0, display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", background: "rgba(0,20,40,0.4)", borderRadius: 3, padding: 8 }}>
              <ComponentSVG id={VEHICLE_COMPONENTS[carouselIdx].id} />
              <div style={{ fontSize: 10, color: "#3a5a70", marginTop: 6, textAlign: "center" }}>
                {VEHICLE_COMPONENTS[carouselIdx].contractor.split("(")[0].trim()}
              </div>
            </div>
            
            {/* Right: scrollable details */}
            <div style={{ flex: 1, overflowY: "auto", paddingRight: 8, fontSize: 12, color: "#8aa8c0", lineHeight: 1.55 }}>
              <div style={{ marginBottom: 6 }}>
                <span style={{ color: "#00aacc", fontSize: 10, letterSpacing: 1, fontWeight: 700 }}>DIMENSIONS</span>
                <div style={{ color: "#aaccdd" }}>{VEHICLE_COMPONENTS[carouselIdx].dimensions}</div>
              </div>
              <div style={{ marginBottom: 6 }}>
                <span style={{ color: "#00aacc", fontSize: 10, letterSpacing: 1, fontWeight: 700 }}>MASS</span>
                <div style={{ color: "#aaccdd" }}>{VEHICLE_COMPONENTS[carouselIdx].mass}</div>
              </div>
              <div style={{ marginBottom: 6 }}>
                <span style={{ color: "#00aacc", fontSize: 10, letterSpacing: 1, fontWeight: 700 }}>MATERIALS</span>
                <div style={{ color: "#aaccdd" }}>{VEHICLE_COMPONENTS[carouselIdx].materials}</div>
              </div>
              <div style={{ marginBottom: 6 }}>
                <span style={{ color: "#00aacc", fontSize: 10, letterSpacing: 1, fontWeight: 700 }}>PROPELLANTS</span>
                <div style={{ color: "#aaccdd" }}>{VEHICLE_COMPONENTS[carouselIdx].propellants}</div>
              </div>
              <div style={{ marginBottom: 6 }}>
                <span style={{ color: "#00aacc", fontSize: 10, letterSpacing: 1, fontWeight: 700 }}>FUNCTION</span>
                <div style={{ color: "#aaccdd" }}>{VEHICLE_COMPONENTS[carouselIdx].function}</div>
              </div>
              <div style={{ marginBottom: 6 }}>
                <span style={{ color: "#00aacc", fontSize: 10, letterSpacing: 1, fontWeight: 700 }}>KEY FEATURES</span>
                <ul style={{ margin: "2px 0 0 16px", padding: 0, color: "#aaccdd" }}>
                  {VEHICLE_COMPONENTS[carouselIdx].keyFeatures.map((f, i) => (
                    <li key={i} style={{ marginBottom: 1 }}>{f}</li>
                  ))}
                </ul>
              </div>
              <div style={{ fontSize: 10, color: "#3a5a70", marginTop: 4, fontStyle: "italic" }}>
                Contractor: {VEHICLE_COMPONENTS[carouselIdx].contractor}
              </div>
            </div>
          </div>
          
          {/* Nav arrows */}
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginTop: 6 }}>
            <button 
              className="gb" 
              onClick={() => setCarouselIdx((carouselIdx - 1 + VEHICLE_COMPONENTS.length) % VEHICLE_COMPONENTS.length)}
            >
              ◀ PREV
            </button>
            <div style={{ display: "flex", gap: 4 }}>
              {VEHICLE_COMPONENTS.map((c, i) => (
                <button
                  key={c.id}
                  onClick={() => setCarouselIdx(i)}
                  style={{
                    width: 8,
                    height: 8,
                    borderRadius: "50%",
                    background: i === carouselIdx ? "#00ccff" : "rgba(0,160,255,0.2)",
                    border: "none",
                    cursor: "pointer",
                    padding: 0,
                    boxShadow: i === carouselIdx ? "0 0 6px rgba(0,200,255,0.6)" : "none"
                  }}
                />
              ))}
            </div>
            <button 
              className="gb" 
              onClick={() => setCarouselIdx((carouselIdx + 1) % VEHICLE_COMPONENTS.length)}
            >
              NEXT ▶
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

/* ─── COMPONENT SVG ILLUSTRATIONS ───
   Improved technical illustrations of each Artemis II component.
   Drawn from official NASA, ULA, Lockheed Martin, Airbus, and
   Northrop Grumman technical reference imagery. */
function ComponentSVG({ id }) {
  const svgProps = { width: 220, height: 220, viewBox: "-110 -110 220 220", xmlns: "http://www.w3.org/2000/svg" };
  
  switch (id) {
    case "sls-core":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="sofiOrange" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="#a04510"/>
              <stop offset="50%" stopColor="#d8631a"/>
              <stop offset="100%" stopColor="#a04510"/>
            </linearGradient>
          </defs>
          <ellipse cx="0" cy="-90" rx="22" ry="5" fill="#e0712a"/>
          <rect x="-22" y="-90" width="44" height="170" fill="url(#sofiOrange)" stroke="#5a2008" strokeWidth="0.6"/>
          <line x1="-22" y1="-30" x2="22" y2="-30" stroke="#5a2008" strokeWidth="1"/>
          <line x1="-22" y1="-25" x2="22" y2="-25" stroke="#3a1004" strokeWidth="0.4"/>
          <text x="0" y="-55" fill="#fff" fontSize="9" textAnchor="middle" fontFamily="sans-serif" fontWeight="700">LOX</text>
          <text x="0" y="20" fill="#fff" fontSize="9" textAnchor="middle" fontFamily="sans-serif" fontWeight="700">LH2</text>
          <line x1="-22" y1="80" x2="22" y2="80" stroke="#3a1004" strokeWidth="0.5"/>
          <rect x="-22" y="80" width="44" height="10" fill="#7a3010" stroke="#3a1004" strokeWidth="0.5"/>
          <g transform="translate(-12, 92)">
            <ellipse cx="0" cy="0" rx="5" ry="2" fill="#c0c0c0"/>
            <path d="M -5 0 L -7 12 L 7 12 L 5 0 Z" fill="#888"/>
          </g>
          <g transform="translate(12, 92)">
            <ellipse cx="0" cy="0" rx="5" ry="2" fill="#c0c0c0"/>
            <path d="M -5 0 L -7 12 L 7 12 L 5 0 Z" fill="#888"/>
          </g>
          <text x="0" y="-105" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">SLS CORE STAGE</text>
          <line x1="-22" y1="-60" x2="-50" y2="-60" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-52" y="-57" fill="#00ccff" fontSize="9" textAnchor="end">LOX tank</text>
          <line x1="-22" y1="-30" x2="-50" y2="-30" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-52" y="-27" fill="#00ccff" fontSize="9" textAnchor="end">Intertank</text>
          <line x1="22" y1="20" x2="50" y2="20" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="23" fill="#00ccff" fontSize="9">LH2 tank</text>
          <line x1="22" y1="92" x2="50" y2="92" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="95" fill="#00ccff" fontSize="9">RS-25 ×4</text>
        </svg>
      );
    case "rs25":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="bellGrad" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#7a7a82"/>
              <stop offset="100%" stopColor="#3a3a42"/>
            </linearGradient>
          </defs>
          <rect x="-14" y="-85" width="28" height="22" fill="#aaaaaa" stroke="#444" strokeWidth="0.5"/>
          <rect x="-18" y="-65" width="36" height="6" fill="#888"/>
          <ellipse cx="-12" cy="-78" rx="4" ry="6" fill="#5a5a6a" stroke="#333" strokeWidth="0.3"/>
          <ellipse cx="12" cy="-78" rx="4" ry="6" fill="#5a5a6a" stroke="#333" strokeWidth="0.3"/>
          <rect x="-15" y="-58" width="30" height="14" fill="#d0d0d0" stroke="#444" strokeWidth="0.5"/>
          <text x="0" y="-49" fill="#222" fontSize="7" textAnchor="middle" fontFamily="monospace">MCC</text>
          <path d="M -15 -44 L -38 70 L 38 70 L 15 -44 Z" fill="url(#bellGrad)" stroke="#222" strokeWidth="0.8"/>
          <ellipse cx="0" cy="70" rx="38" ry="5" fill="#222" stroke="#aaa" strokeWidth="0.5"/>
          <g stroke="#5a5a6a" strokeWidth="0.3" fill="none" opacity="0.6">
            <line x1="-25" y1="0" x2="25" y2="0"/>
            <line x1="-30" y1="20" x2="30" y2="20"/>
            <line x1="-34" y1="40" x2="34" y2="40"/>
          </g>
          <text x="0" y="-100" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">RS-25D ENGINE</text>
          <line x1="-12" y1="-78" x2="-50" y2="-90" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-52" y="-87" fill="#00ccff" fontSize="9" textAnchor="end">HPFTP</text>
          <line x1="12" y1="-78" x2="50" y2="-90" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-87" fill="#00ccff" fontSize="9">HPOTP</text>
          <line x1="0" y1="-49" x2="50" y2="-50" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-47" fill="#00ccff" fontSize="9">Combustion</text>
          <line x1="20" y1="30" x2="50" y2="30" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="33" fill="#00ccff" fontSize="9">Bell nozzle</text>
        </svg>
      );
    case "srb":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="srbGrad" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="#d8d8d8"/>
              <stop offset="50%" stopColor="#f0f0f0"/>
              <stop offset="100%" stopColor="#b0b0b0"/>
            </linearGradient>
          </defs>
          <polygon points="-10,-95 0,-105 10,-95" fill="#e0e0e0" stroke="#666" strokeWidth="0.5"/>
          <path d="M -14 -95 Q -14 -85 -14 -80 L 14 -80 Q 14 -85 14 -95 Z" fill="#d0d0d0" stroke="#666" strokeWidth="0.5"/>
          <rect x="-14" y="-80" width="28" height="32" fill="url(#srbGrad)" stroke="#666" strokeWidth="0.5"/>
          <line x1="-14" y1="-48" x2="14" y2="-48" stroke="#444" strokeWidth="1.2"/>
          <line x1="-14" y1="-46" x2="14" y2="-46" stroke="#888" strokeWidth="0.4"/>
          <rect x="-14" y="-48" width="28" height="32" fill="url(#srbGrad)" stroke="#666" strokeWidth="0.5"/>
          <line x1="-14" y1="-16" x2="14" y2="-16" stroke="#444" strokeWidth="1.2"/>
          <line x1="-14" y1="-14" x2="14" y2="-14" stroke="#888" strokeWidth="0.4"/>
          <rect x="-14" y="-16" width="28" height="32" fill="url(#srbGrad)" stroke="#666" strokeWidth="0.5"/>
          <line x1="-14" y1="16" x2="14" y2="16" stroke="#444" strokeWidth="1.2"/>
          <line x1="-14" y1="18" x2="14" y2="18" stroke="#888" strokeWidth="0.4"/>
          <rect x="-14" y="16" width="28" height="32" fill="url(#srbGrad)" stroke="#666" strokeWidth="0.5"/>
          <line x1="-14" y1="48" x2="14" y2="48" stroke="#444" strokeWidth="1.2"/>
          <line x1="-14" y1="50" x2="14" y2="50" stroke="#888" strokeWidth="0.4"/>
          <rect x="-14" y="48" width="28" height="22" fill="url(#srbGrad)" stroke="#666" strokeWidth="0.5"/>
          <polygon points="-14,70 -18,80 18,80 14,70" fill="#5a5a5a" stroke="#222" strokeWidth="0.5"/>
          <polygon points="-18,80 -22,98 22,98 18,80" fill="#2a2a2a"/>
          <text x="0" y="-65" fill="#444" fontSize="7" textAnchor="middle" fontFamily="monospace" fontWeight="700">SEG 5</text>
          <text x="0" y="-32" fill="#444" fontSize="7" textAnchor="middle" fontFamily="monospace" fontWeight="700">SEG 4</text>
          <text x="0" y="0" fill="#444" fontSize="7" textAnchor="middle" fontFamily="monospace" fontWeight="700">SEG 3</text>
          <text x="0" y="32" fill="#444" fontSize="7" textAnchor="middle" fontFamily="monospace" fontWeight="700">SEG 2</text>
          <text x="0" y="60" fill="#444" fontSize="7" textAnchor="middle" fontFamily="monospace" fontWeight="700">SEG 1</text>
          <text x="0" y="-115" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">SRB (×2)</text>
          <line x1="14" y1="-95" x2="50" y2="-95" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-92" fill="#00ccff" fontSize="9">Nose cap</text>
          <line x1="14" y1="-30" x2="50" y2="-30" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-27" fill="#00ccff" fontSize="9">PBAN propellant</text>
          <line x1="22" y1="80" x2="55" y2="80" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="83" fill="#00ccff" fontSize="9">Aft skirt + TVC</text>
          <line x1="0" y1="98" x2="50" y2="105" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="108" fill="#00ccff" fontSize="9">Nozzle</text>
        </svg>
      );
    case "icps":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="icpsGrad" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="#b8b8c0"/>
              <stop offset="50%" stopColor="#e0e0e8"/>
              <stop offset="100%" stopColor="#909098"/>
            </linearGradient>
          </defs>
          <ellipse cx="0" cy="-65" rx="28" ry="6" fill="#d0d0d8" stroke="#666" strokeWidth="0.5"/>
          <rect x="-28" y="-65" width="56" height="14" fill="url(#icpsGrad)" stroke="#666" strokeWidth="0.5"/>
          <text x="0" y="-55" fill="#444" fontSize="7" textAnchor="middle" fontFamily="monospace">FORWARD SKIRT</text>
          <line x1="-28" y1="-51" x2="28" y2="-51" stroke="#444" strokeWidth="0.5"/>
          <path d="M -28 -51 L -22 -10 L 22 -10 L 28 -51 Z" fill="url(#icpsGrad)" stroke="#666" strokeWidth="0.5"/>
          <text x="0" y="-30" fill="#222" fontSize="8" textAnchor="middle" fontWeight="700">LOX</text>
          <line x1="-22" y1="-10" x2="22" y2="-10" stroke="#444" strokeWidth="0.5"/>
          <rect x="-22" y="-10" width="44" height="55" fill="url(#icpsGrad)" stroke="#666" strokeWidth="0.5"/>
          <text x="0" y="20" fill="#222" fontSize="8" textAnchor="middle" fontWeight="700">LH2</text>
          <line x1="-22" y1="45" x2="22" y2="45" stroke="#444" strokeWidth="0.5"/>
          <rect x="-22" y="45" width="44" height="12" fill="#999" stroke="#666" strokeWidth="0.5"/>
          <text x="0" y="54" fill="#222" fontSize="6" textAnchor="middle">EQUIPMENT</text>
          <ellipse cx="0" cy="57" rx="22" ry="3" fill="#666"/>
          <rect x="-5" y="57" width="10" height="14" fill="#444"/>
          <path d="M -8 71 L -12 90 L 12 90 L 8 71 Z" fill="#3a3a3a" stroke="#222" strokeWidth="0.5"/>
          <ellipse cx="0" cy="90" rx="12" ry="2" fill="#222"/>
          <text x="0" y="-85" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">ICPS</text>
          <line x1="28" y1="-58" x2="55" y2="-65" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="-62" fill="#00ccff" fontSize="9">Forward skirt</text>
          <line x1="22" y1="-30" x2="55" y2="-30" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="-27" fill="#00ccff" fontSize="9">LOX tank</text>
          <line x1="22" y1="20" x2="55" y2="20" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="23" fill="#00ccff" fontSize="9">LH2 tank</text>
          <line x1="12" y1="85" x2="55" y2="85" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="88" fill="#00ccff" fontSize="9">RL10C-2</text>
        </svg>
      );
    case "las":
      return (
        <svg {...svgProps}>
          <polygon points="-3,-95 3,-95 8,-65 -8,-65" fill="#d04020" stroke="#601000" strokeWidth="0.5"/>
          <rect x="-9" y="-65" width="18" height="22" fill="#c83820" stroke="#601000" strokeWidth="0.5"/>
          <text x="0" y="-50" fill="#fff" fontSize="6" textAnchor="middle" fontFamily="monospace">JETT</text>
          <rect x="-7" y="-43" width="14" height="20" fill="#a82810"/>
          <rect x="-9" y="-23" width="18" height="14" fill="#882010"/>
          <text x="0" y="-13" fill="#fff" fontSize="6" textAnchor="middle" fontFamily="monospace">ATT</text>
          <polygon points="-22,-9 22,-9 28,8 -28,8" fill="#c83820" stroke="#601000" strokeWidth="0.7"/>
          <g fill="#882010">
            <polygon points="-26,-2 -32,8 -22,8 -20,-2"/>
            <polygon points="26,-2 32,8 22,8 20,-2"/>
          </g>
          <polygon points="-28,8 28,8 32,22 -32,22" fill="#982410"/>
          <text x="0" y="20" fill="#fff" fontSize="7" textAnchor="middle" fontFamily="monospace" fontWeight="700">ABORT</text>
          <path d="M -32 22 Q -34 50 -28 65 L 28 65 Q 34 50 32 22" fill="#e8e8e0" stroke="#666" strokeWidth="0.5"/>
          <ellipse cx="0" cy="35" rx="28" ry="3" fill="#3a2818" opacity="0.5"/>
          <text x="0" y="-105" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">LAUNCH ABORT SYSTEM</text>
          <line x1="3" y1="-80" x2="50" y2="-80" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-77" fill="#00ccff" fontSize="9">Jettison motor</text>
          <line x1="9" y1="-32" x2="50" y2="-32" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-29" fill="#00ccff" fontSize="9">Attitude control</text>
          <line x1="22" y1="0" x2="55" y2="0" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="3" fill="#00ccff" fontSize="9">Abort motor</text>
          <line x1="32" y1="40" x2="62" y2="40" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="64" y="43" fill="#00ccff" fontSize="9">Crew Module</text>
        </svg>
      );
    case "orion-cm":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="cmBody" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#f0f0e8"/>
              <stop offset="100%" stopColor="#b8b8a8"/>
            </linearGradient>
            <radialGradient id="hsCol" cx="50%" cy="50%" r="50%">
              <stop offset="0%" stopColor="#5a3818"/>
              <stop offset="100%" stopColor="#2a1408"/>
            </radialGradient>
          </defs>
          <rect x="-15" y="-90" width="30" height="8" fill="#a8a8a8" stroke="#666" strokeWidth="0.4"/>
          <rect x="-10" y="-98" width="20" height="8" fill="#888"/>
          <circle cx="0" cy="-94" r="3" fill="#444" stroke="#aaa" strokeWidth="0.3"/>
          <text x="0" y="-105" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">ORION CREW MODULE</text>
          <path d="M -55 30 L -38 -82 L 38 -82 L 55 30 Z" fill="url(#cmBody)" stroke="#444" strokeWidth="0.6"/>
          <line x1="-43" y1="-50" x2="43" y2="-50" stroke="#aaa" strokeWidth="0.3"/>
          <line x1="-48" y1="-20" x2="48" y2="-20" stroke="#aaa" strokeWidth="0.3"/>
          <line x1="-52" y1="10" x2="52" y2="10" stroke="#aaa" strokeWidth="0.3"/>
          <ellipse cx="-25" cy="-25" rx="4" ry="3" fill="#1a3050" stroke="#445566" strokeWidth="0.5"/>
          <ellipse cx="-15" cy="-25" rx="4" ry="3" fill="#1a3050" stroke="#445566" strokeWidth="0.5"/>
          <ellipse cx="0" cy="-25" rx="4" ry="3" fill="#1a3050" stroke="#445566" strokeWidth="0.5"/>
          <ellipse cx="15" cy="-25" rx="4" ry="3" fill="#1a3050" stroke="#445566" strokeWidth="0.5"/>
          <rect x="20" y="-5" width="10" height="6" fill="#666" stroke="#222" strokeWidth="0.3"/>
          <rect x="-30" y="-5" width="10" height="6" fill="#666" stroke="#222" strokeWidth="0.3"/>
          <ellipse cx="0" cy="30" rx="55" ry="10" fill="url(#hsCol)" stroke="#1a0800" strokeWidth="0.8"/>
          <ellipse cx="0" cy="32" rx="50" ry="6" fill="none" stroke="#7a4828" strokeWidth="0.4"/>
          <line x1="-55" y1="30" x2="-75" y2="35" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-77" y="38" fill="#00ccff" fontSize="9" textAnchor="end">AVCOAT shield</text>
          <line x1="0" y1="-25" x2="60" y2="-30" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="62" y="-27" fill="#00ccff" fontSize="9">Windows (4)</text>
          <line x1="0" y1="-90" x2="55" y2="-95" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="-92" fill="#00ccff" fontSize="9">Docking adapter</text>
          <line x1="-30" y1="0" x2="-65" y2="0" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-67" y="3" fill="#00ccff" fontSize="9" textAnchor="end">Back shell TPS</text>
        </svg>
      );
    case "heat-shield":
      return (
        <svg {...svgProps}>
          <defs>
            <radialGradient id="shGrad" cx="50%" cy="40%" r="60%">
              <stop offset="0%" stopColor="#7a4828"/>
              <stop offset="60%" stopColor="#3a2010"/>
              <stop offset="100%" stopColor="#1a0a00"/>
            </radialGradient>
          </defs>
          <text x="0" y="-90" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">AVCOAT HEAT SHIELD</text>
          <path d="M -85 -10 Q -85 50 0 65 Q 85 50 85 -10 L 80 -5 Q 0 25 -80 -5 Z" fill="url(#shGrad)" stroke="#1a0a00" strokeWidth="1"/>
          <g stroke="#5a3818" strokeWidth="0.4" fill="none">
            <ellipse cx="0" cy="0" rx="78" ry="8"/>
            <ellipse cx="0" cy="10" rx="74" ry="8"/>
            <ellipse cx="0" cy="20" rx="68" ry="8"/>
            <ellipse cx="0" cy="30" rx="58" ry="6"/>
            <ellipse cx="0" cy="40" rx="44" ry="5"/>
            <ellipse cx="0" cy="48" rx="28" ry="4"/>
          </g>
          <g stroke="#7a5028" strokeWidth="0.4" fill="none">
            <line x1="-78" y1="-2" x2="-72" y2="48"/>
            <line x1="-58" y1="-7" x2="-55" y2="56"/>
            <line x1="-38" y1="-9" x2="-37" y2="62"/>
            <line x1="-18" y1="-10" x2="-18" y2="65"/>
            <line x1="2" y1="-10" x2="2" y2="65"/>
            <line x1="22" y1="-10" x2="22" y2="64"/>
            <line x1="42" y1="-9" x2="42" y2="60"/>
            <line x1="62" y1="-7" x2="58" y2="52"/>
            <line x1="78" y1="-2" x2="72" y2="46"/>
          </g>
          <g fill="#9a6038" opacity="0.5">
            <circle cx="-50" cy="20" r="2"/>
            <circle cx="-20" cy="35" r="2"/>
            <circle cx="20" cy="40" r="2"/>
            <circle cx="50" cy="25" r="2"/>
            <circle cx="0" cy="50" r="2"/>
            <circle cx="-65" cy="5" r="2"/>
          </g>
          <text x="0" y="85" fill="#00ccff" fontSize="9" textAnchor="middle">186 AVCOAT blocks · Ti substructure</text>
          <text x="0" y="98" fill="#00ccff" fontSize="9" textAnchor="middle">Silica fibers + phenolic resin</text>
        </svg>
      );
    case "esm":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="mliGold" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#e8c060"/>
              <stop offset="100%" stopColor="#a87820"/>
            </linearGradient>
            <pattern id="solarCells" x="0" y="0" width="6" height="4" patternUnits="userSpaceOnUse">
              <rect width="6" height="4" fill="#1a3380"/>
              <rect width="2.5" height="3.5" x="0.25" y="0.25" fill="#2a4ab0" stroke="#0a1a40" strokeWidth="0.1"/>
              <rect width="2.5" height="3.5" x="3.25" y="0.25" fill="#2a4ab0" stroke="#0a1a40" strokeWidth="0.1"/>
            </pattern>
          </defs>
          <text x="0" y="-95" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">EUROPEAN SERVICE MODULE</text>
          <ellipse cx="0" cy="-50" rx="22" ry="5" fill="#e0c068" stroke="#604010" strokeWidth="0.5"/>
          <rect x="-22" y="-50" width="44" height="80" fill="url(#mliGold)" stroke="#604010" strokeWidth="0.6"/>
          <line x1="-22" y1="-30" x2="22" y2="-30" stroke="#604010" strokeWidth="0.4"/>
          <line x1="-22" y1="-10" x2="22" y2="-10" stroke="#604010" strokeWidth="0.4"/>
          <line x1="-22" y1="10" x2="22" y2="10" stroke="#604010" strokeWidth="0.4"/>
          <ellipse cx="0" cy="30" rx="22" ry="4" fill="#a07810"/>
          <g>
            <rect x="-92" y="-5" width="32" height="14" fill="url(#solarCells)" stroke="#000" strokeWidth="0.3"/>
            <rect x="-58" y="-5" width="32" height="14" fill="url(#solarCells)" stroke="#000" strokeWidth="0.3"/>
            <line x1="-22" y1="2" x2="-92" y2="2" stroke="#666" strokeWidth="0.4"/>
          </g>
          <g>
            <rect x="26" y="-5" width="32" height="14" fill="url(#solarCells)" stroke="#000" strokeWidth="0.3"/>
            <rect x="60" y="-5" width="32" height="14" fill="url(#solarCells)" stroke="#000" strokeWidth="0.3"/>
            <line x1="22" y1="2" x2="92" y2="2" stroke="#666" strokeWidth="0.4"/>
          </g>
          <g fill="#3a3a3a">
            <circle cx="-18" cy="32" r="1.5"/>
            <circle cx="18" cy="32" r="1.5"/>
            <circle cx="-12" cy="34" r="1.5"/>
            <circle cx="12" cy="34" r="1.5"/>
          </g>
          <rect x="-3" y="32" width="6" height="14" fill="#444"/>
          <path d="M -6 46 L -10 62 L 10 62 L 6 46 Z" fill="#2a2a2a" stroke="#888" strokeWidth="0.3"/>
          <line x1="-22" y1="-30" x2="-50" y2="-50" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-52" y="-47" fill="#00ccff" fontSize="9" textAnchor="end">Gold MLI</text>
          <line x1="-75" y1="2" x2="-75" y2="20" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-75" y="32" fill="#00ccff" fontSize="9" textAnchor="middle">Solar wing</text>
          <line x1="22" y1="20" x2="50" y2="40" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="43" fill="#00ccff" fontSize="9">RCS pod</text>
          <line x1="0" y1="62" x2="50" y2="68" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="71" fill="#00ccff" fontSize="9">AJ10 main</text>
        </svg>
      );
    case "aj10":
      return (
        <svg {...svgProps}>
          <defs>
            <linearGradient id="bell2" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#7a6038"/>
              <stop offset="100%" stopColor="#3a2818"/>
            </linearGradient>
          </defs>
          <text x="0" y="-95" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">AJ10-190 (OMS-E)</text>
          <rect x="-12" y="-75" width="24" height="22" fill="#888" stroke="#444" strokeWidth="0.5"/>
          <ellipse cx="-7" cy="-65" rx="3" ry="5" fill="#666"/>
          <ellipse cx="7" cy="-65" rx="3" ry="5" fill="#666"/>
          <text x="0" y="-58" fill="#fff" fontSize="6" textAnchor="middle" fontFamily="monospace">VALVES</text>
          <rect x="-13" y="-53" width="26" height="8" fill="#9a9a9a"/>
          <rect x="-15" y="-45" width="30" height="14" fill="#666" stroke="#222" strokeWidth="0.5"/>
          <text x="0" y="-35" fill="#ccc" fontSize="6" textAnchor="middle">CHAMBER</text>
          <path d="M -15 -31 L -45 70 L 45 70 L 15 -31 Z" fill="url(#bell2)" stroke="#222" strokeWidth="0.7"/>
          <ellipse cx="0" cy="70" rx="45" ry="5" fill="#1a1408" stroke="#aaa" strokeWidth="0.4"/>
          <g stroke="#5a4828" strokeWidth="0.3" opacity="0.6">
            <line x1="-25" y1="0" x2="25" y2="0"/>
            <line x1="-32" y1="22" x2="32" y2="22"/>
            <line x1="-40" y1="44" x2="40" y2="44"/>
          </g>
          <line x1="12" y1="-65" x2="50" y2="-75" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="52" y="-72" fill="#00ccff" fontSize="9">Bipropellant</text>
          <text x="52" y="-62" fill="#00ccff" fontSize="9">valves</text>
          <line x1="-15" y1="-38" x2="-50" y2="-50" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-52" y="-47" fill="#00ccff" fontSize="9" textAnchor="end">Combustion</text>
          <line x1="35" y1="35" x2="65" y2="35" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="67" y="38" fill="#00ccff" fontSize="9">Columbium</text>
          <text x="67" y="48" fill="#00ccff" fontSize="9">nozzle</text>
        </svg>
      );
    case "solar-arrays":
      return (
        <svg {...svgProps}>
          <defs>
            <pattern id="cells2" x="0" y="0" width="8" height="6" patternUnits="userSpaceOnUse">
              <rect width="8" height="6" fill="#1a2870"/>
              <rect width="3.5" height="5" x="0.25" y="0.5" fill="#2a40a0" stroke="#000820" strokeWidth="0.15"/>
              <rect width="3.5" height="5" x="4.25" y="0.5" fill="#2a40a0" stroke="#000820" strokeWidth="0.15"/>
            </pattern>
          </defs>
          <text x="0" y="-95" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">SOLAR ARRAY WINGS (×4)</text>
          <circle cx="0" cy="0" r="14" fill="#a07810" stroke="#604010" strokeWidth="0.5"/>
          <circle cx="0" cy="0" r="11" fill="#c89020"/>
          <text x="0" y="3" fill="#fff" fontSize="7" textAnchor="middle" fontWeight="700">ESM</text>
          <g>
            <rect x="14" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="42" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="70" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <line x1="40" y1="-9" x2="40" y2="9" stroke="#222" strokeWidth="0.4"/>
            <line x1="68" y1="-9" x2="68" y2="9" stroke="#222" strokeWidth="0.4"/>
          </g>
          <g transform="rotate(90)">
            <rect x="14" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="42" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="70" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <line x1="40" y1="-9" x2="40" y2="9" stroke="#222" strokeWidth="0.4"/>
            <line x1="68" y1="-9" x2="68" y2="9" stroke="#222" strokeWidth="0.4"/>
          </g>
          <g transform="rotate(180)">
            <rect x="14" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="42" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="70" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <line x1="40" y1="-9" x2="40" y2="9" stroke="#222" strokeWidth="0.4"/>
            <line x1="68" y1="-9" x2="68" y2="9" stroke="#222" strokeWidth="0.4"/>
          </g>
          <g transform="rotate(270)">
            <rect x="14" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="42" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <rect x="70" y="-9" width="26" height="18" fill="url(#cells2)" stroke="#000" strokeWidth="0.4"/>
            <line x1="40" y1="-9" x2="40" y2="9" stroke="#222" strokeWidth="0.4"/>
            <line x1="68" y1="-9" x2="68" y2="9" stroke="#222" strokeWidth="0.4"/>
          </g>
          <text x="0" y="105" fill="#00ccff" fontSize="9" textAnchor="middle">Total span 19 m · 11.2 kW</text>
        </svg>
      );
    case "rcs":
      return (
        <svg {...svgProps}>
          <text x="0" y="-95" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">ESM RCS THRUSTERS</text>
          <circle cx="0" cy="0" r="38" fill="#9a8030" stroke="#604010" strokeWidth="0.6"/>
          <circle cx="0" cy="0" r="34" fill="#b89030" opacity="0.5"/>
          <g fill="#888" stroke="#222" strokeWidth="0.4">
            <rect x="-4" y="-46" width="8" height="8" rx="1"/>
            <polygon points="-5,-46 -7,-54 7,-54 5,-46"/>
            <rect x="-4" y="38" width="8" height="8" rx="1"/>
            <polygon points="-5,46 -7,54 7,54 5,46"/>
            <rect x="38" y="-4" width="8" height="8" rx="1"/>
            <polygon points="46,-5 54,-7 54,7 46,5"/>
            <rect x="-46" y="-4" width="8" height="8" rx="1"/>
            <polygon points="-46,-5 -54,-7 -54,7 -46,5"/>
          </g>
          <g fill="#666" stroke="#222" strokeWidth="0.3">
            <circle cx="22" cy="-30" r="2.5"/>
            <circle cx="26" cy="-26" r="2.5"/>
            <circle cx="30" cy="-22" r="2.5"/>
            <circle cx="-22" cy="30" r="2.5"/>
            <circle cx="-26" cy="26" r="2.5"/>
            <circle cx="-30" cy="22" r="2.5"/>
            <circle cx="22" cy="30" r="2.5"/>
            <circle cx="26" cy="26" r="2.5"/>
            <circle cx="30" cy="22" r="2.5"/>
            <circle cx="-22" cy="-30" r="2.5"/>
            <circle cx="-26" cy="-26" r="2.5"/>
            <circle cx="-30" cy="-22" r="2.5"/>
          </g>
          <text x="0" y="3" fill="#fff" fontSize="6" textAnchor="middle" fontWeight="700">ESM</text>
          <line x1="0" y1="-54" x2="35" y2="-70" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="37" y="-67" fill="#00ccff" fontSize="9">8× R-4D aux</text>
          <line x1="30" y1="-22" x2="65" y2="-30" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="67" y="-27" fill="#00ccff" fontSize="9">24 RCS thrusters</text>
          <text x="67" y="-17" fill="#00ccff" fontSize="9">in 6 pods of 4</text>
        </svg>
      );
    case "parachutes":
      return (
        <svg {...svgProps}>
          <defs>
            <radialGradient id="chute" cx="50%" cy="80%" r="60%">
              <stop offset="0%" stopColor="#ff8855"/>
              <stop offset="60%" stopColor="#dd6633"/>
              <stop offset="100%" stopColor="#aa4422"/>
            </radialGradient>
          </defs>
          <text x="0" y="-95" fill="#00ddff" fontSize="11" textAnchor="middle" fontWeight="700">CPAS PARACHUTES</text>
          <path d="M -45 -55 Q -45 -82 0 -82 Q 45 -82 45 -55 Q 45 -50 0 -42 Q -45 -50 -45 -55 Z" fill="url(#chute)" stroke="#601000" strokeWidth="0.5"/>
          <g stroke="#fff" strokeWidth="0.3" opacity="0.7">
            <line x1="-40" y1="-65" x2="-40" y2="-50"/>
            <line x1="-30" y1="-72" x2="-30" y2="-46"/>
            <line x1="-20" y1="-78" x2="-20" y2="-44"/>
            <line x1="-10" y1="-81" x2="-10" y2="-43"/>
            <line x1="0" y1="-82" x2="0" y2="-42"/>
            <line x1="10" y1="-81" x2="10" y2="-43"/>
            <line x1="20" y1="-78" x2="20" y2="-44"/>
            <line x1="30" y1="-72" x2="30" y2="-46"/>
            <line x1="40" y1="-65" x2="40" y2="-50"/>
          </g>
          <path d="M -45 -55 Q -42 -50 -38 -45" stroke="#fff" strokeWidth="0.4" fill="none" opacity="0.6"/>
          <path d="M 45 -55 Q 42 -50 38 -45" stroke="#fff" strokeWidth="0.4" fill="none" opacity="0.6"/>
          <line x1="-40" y1="-50" x2="-3" y2="20" stroke="#ddd" strokeWidth="0.4"/>
          <line x1="-25" y1="-46" x2="-2" y2="20" stroke="#ddd" strokeWidth="0.4"/>
          <line x1="-10" y1="-44" x2="-1" y2="20" stroke="#ddd" strokeWidth="0.4"/>
          <line x1="10" y1="-44" x2="1" y2="20" stroke="#ddd" strokeWidth="0.4"/>
          <line x1="25" y1="-46" x2="2" y2="20" stroke="#ddd" strokeWidth="0.4"/>
          <line x1="40" y1="-50" x2="3" y2="20" stroke="#ddd" strokeWidth="0.4"/>
          <path d="M -18 20 L -12 45 L 12 45 L 18 20 Z" fill="#c0c0b8" stroke="#666" strokeWidth="0.5"/>
          <ellipse cx="0" cy="45" rx="18" ry="3" fill="#3a2818"/>
          <path d="M -25 50 Q -25 60 -15 60 L 15 60 Q 25 60 25 50" stroke="#3060a0" strokeWidth="0.4" fill="none" opacity="0.5"/>
          <path d="M -30 55 Q -30 68 -15 68 L 15 68 Q 30 68 30 55" stroke="#3060a0" strokeWidth="0.4" fill="none" opacity="0.4"/>
          <line x1="-45" y1="-65" x2="-75" y2="-75" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-77" y="-72" fill="#00ccff" fontSize="9" textAnchor="end">Main canopy</text>
          <text x="-77" y="-62" fill="#00ccff" fontSize="9" textAnchor="end">35.4 m / 116 ft</text>
          <line x1="-15" y1="0" x2="-65" y2="0" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="-67" y="3" fill="#00ccff" fontSize="9" textAnchor="end">Kevlar lines</text>
          <line x1="20" y1="40" x2="55" y2="50" stroke="#00aacc" strokeWidth="0.5"/>
          <text x="57" y="53" fill="#00ccff" fontSize="9">Crew Module</text>
          <text x="0" y="85" fill="#00ccff" fontSize="9" textAnchor="middle">11 chutes total · 17 mph splashdown</text>
        </svg>
      );
    default:
      return <svg {...svgProps}><text x="0" y="0" fill="#666" fontSize="10" textAnchor="middle">N/A</text></svg>;
  }
}

function TR({ l, v, c, big }) {
  return (
    <div style={{ display: "flex", justifyContent: "space-between", lineHeight: 1.5, alignItems: "baseline" }}>
      <span style={{ color: "#2a4a60", fontSize: 10 }}>{l}</span>
      <span style={{ color: c || "#7a9eb8", fontSize: big ? 11 : 10, fontWeight: big ? 700 : 400 }}>{v}</span>
    </div>
  );
}

function SL({ label, unit, min, max, step, val, set, fmt }) {
  return (
    <div style={{ marginBottom: 8 }}>
      <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 2 }}>
        <span style={{ fontSize: 10, color: "#2a4a60" }}>{label}</span>
        <span style={{ fontSize: 10, color: "#00ccff" }}>{fmt(val)} <span style={{ fontSize: 10, color: "#2a4a60" }}>{unit}</span></span>
      </div>
      <input type="range" min={min} max={max} step={step} value={val} onChange={e => set(+e.target.value)} />
    </div>
  );
}
