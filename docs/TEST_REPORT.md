# FkToIkBake engine test report

Run on: Linux x86-64, g++ 13.3, `-O2`, single thread. Built with `cmake` (`fk2ik_tests` target).
**Maya was not involved.** These results validate the engine math. The Maya layer still has to be validated in Maya (see README checklist).

## Method

`tests/SyntheticRig.*` builds a two-bone rig independently of the engine:

* An FK chain of joints with `jointOrient` and animated rotations, under a root that translates and spins through 360°.
* An IK chain with the same rest structure, solved by its own analytic Rotate-Plane solver (pole vector + handle twist, elbow bends toward the pole). The IK end joint is orient-constrained to the IK control (maintain offset), and the handle is parented under the control.
* An IK control and a pole control under animated parent spaces, with non-default rotate order, rotateAxis, pivots, parent scale, and in one rig a joint as the IK control.

For each case the engine calibrates on the IK side at frame 0 and bakes frames 1–2000. Then the baked channel values are pushed back through the control transforms and the reference IK solver. The resulting IK joints are compared against the FK joints at every sampled frame.

Rigs: `arm_L` (X down the bone), `arm_R` (mirrored, ZXY order, rotateAxis, pivots, parent scale 1.3, handle twist 23°), `leg_L` (Y down, YZX), `leg_R` (−Y down, IK control is a joint with jointOrient, XZY).

Animations:
* `normal`: everyday motion.
* `fast`: up to about 25–30° per frame on several axes.
* `spin360`: continuous spins crossing 180° and 360° many times.
* `straight`: mocap-like, 0–1.7° of bend, including exactly straight.
* `offHinge`: the animator also rotated the FK elbow off its hinge axis.

`flips` = frames where the chosen Euler triple is farther from the previous key than the best equivalent solution found by brute force (both solutions, ±3 turns on every axis). `>90/f` = channel moves larger than 90° in one frame. These are inherent near gimbal lock at this speed and are already the optimal choice. `fallbk` = samples where position mode used the straight-limb fallback.

## Results summary

* **Wrist/ankle position:** ≤ 6e-13 units in every case.
* **Wrist/ankle orientation:** ≤ 1.7e-6° in every case. That floor is the `acos` resolution of the measurement, not the solve.
* **Elbow/knee position:**
  * ≤ 3e-12 units for all planar FK animation, in both modes.
  * The near-straight case is ≤ 4.2e-6 units, limited by the reference solver's law-of-cosines precision.
* **Elbow/knee orientation:** ≤ 1.8e-6° for planar FK animation, ≤ 8.4e-6° when near-straight.
* **Off-hinge FK elbow** (a limit of any Rotate-Plane IK):
  * Position mode keeps the elbow position exact (≤ 4e-12 units). The elbow twist error is 27.8° on arms and 1.7° on legs.
  * Rotation mode reduces the twist error to 10.0° / 0.12°, at the cost of 4.1 / 0.065 units of elbow position.
* **Euler flips:** 0 in all 40 combinations and in the 100k-frame run.
* **Sampling:** step 0.5, 2 and 5 are exact at the sampled frames.
* **Long sequences:** 100,000 frames, with max error 3.7e-11 units and 0 flips.

Performance (engine only):

* About 1.5 µs per limb-sample for match + channel solve. 100k frames × 4 limbs takes about 0.6 s.
* Including synthetic rig evaluation, about 4.4 µs per frame.
* In Maya, the total will be dominated by DG evaluation of the rig per sample, not by the engine.

Key reduction (`arm_R`, 2000 frames, 18,000 dense keys):

| tolerance (units / deg) | keys | max wrist error |
|---|---|---|
| 0.001 | 17,127 (95%) | 0.0013 |
| 0.01 | 13,254 (74%) | 0.0193 |
| 0.05 | 9,441 (52%) | 0.109 |

The tolerance applies per channel, so the wrist error can be up to about 2× the tolerance because of lever arms. Reduction is off by default.

## Full output

```
FkToIkBake engine validation
============================
[unit] Euler <-> matrix, all rotate orders (incl. gimbal)
  worst roundtrip error 1.71e-06 deg
  closestEuler: orientation kept (1.7e-06 deg), jumps: 0 / 120000
[unit] control channel solve (pivots, rotateAxis, shear, jointOrient, parent space)
  worst world-matrix element error 8.53e-14, euler deviation 3.23e-13 rad
[unit] key reducer + rig map parser
  sine 1001 samples -> 667 keys at tol 0.01 (max err 0.0100)
  incomplete map rejected with: limb 'x' is missing: fkMid, fkEnd, ikUpper, ikMid, ikEnd, ikControl, poleControl

[rig] FK -> IK bake, frames 1..2000, step 1.  Errors are max over all frames,
      measured on the IK chain re-solved from the baked keys vs. the FK chain.
      units: scene units (limb ~60-90 units long) and degrees

rig                                          anim      mode     | wristPos  elbowPos  wristRot  elbowRot  | flips  >90/f fallbk
arm_L  (X-down, ctrl XYZ)                    normal    position |  3.42e-13  9.55e-13  1.21e-06  1.21e-06 |     0      0      0
arm_L  (X-down, ctrl XYZ)                    normal    rotation |  3.42e-13  9.55e-13  1.21e-06  1.71e-06 |     0      0      0
arm_L  (X-down, ctrl XYZ)                    fast      position |  3.50e-13  1.29e-12  1.21e-06  1.71e-06 |     0     13      0
arm_L  (X-down, ctrl XYZ)                    fast      rotation |  3.50e-13  1.29e-12  1.21e-06  1.71e-06 |     0     13      0
arm_L  (X-down, ctrl XYZ)                    spin360   position |  3.55e-13  7.76e-13  1.71e-06  1.71e-06 |     0      0      0
arm_L  (X-down, ctrl XYZ)                    spin360   rotation |  3.55e-13  7.72e-13  1.71e-06  1.71e-06 |     0      0      0
arm_L  (X-down, ctrl XYZ)                    straight  position |  3.43e-13  2.53e-06  1.21e-06  5.12e-06 |     0      0   1589
arm_L  (X-down, ctrl XYZ)                    straight  rotation |  3.43e-13  2.53e-06  1.21e-06  5.12e-06 |     0      0      0
arm_L  (X-down, ctrl XYZ)                    offHinge  position |  3.53e-13  6.03e-13  1.21e-06  2.78e+01 |     0      0      0
arm_L  (X-down, ctrl XYZ)                    offHinge  rotation |  3.53e-13  4.07e+00  1.21e-06  9.98e+00 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) normal    position |  4.84e-13  1.26e-12  1.21e-06  1.21e-06 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) normal    rotation |  4.84e-13  1.26e-12  1.21e-06  1.71e-06 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) fast      position |  5.75e-13  1.33e-12  1.21e-06  1.71e-06 |     0     11      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) fast      rotation |  5.75e-13  1.34e-12  1.21e-06  1.71e-06 |     0     11      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) spin360   position |  5.71e-13  1.24e-12  1.21e-06  1.71e-06 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) spin360   rotation |  5.71e-13  1.24e-12  1.21e-06  1.71e-06 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) straight  position |  5.80e-13  4.12e-06  1.21e-06  8.37e-06 |     0      0   1589
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) straight  rotation |  5.80e-13  4.12e-06  1.21e-06  8.37e-06 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) offHinge  position |  5.71e-13  8.80e-13  1.21e-06  2.78e+01 |     0      0      0
arm_R  (mirrored, ZXY+RA+pivot+scale, twist) offHinge  rotation |  5.71e-13  4.07e+00  1.21e-06  9.98e+00 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    normal    position |  2.80e-13  1.84e-12  1.21e-06  1.21e-06 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    normal    rotation |  2.80e-13  1.84e-12  1.21e-06  1.21e-06 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    fast      position |  3.27e-13  1.46e-12  1.21e-06  1.21e-06 |     0      8      0
leg_L  (Y-down, ctrl YZX)                    fast      rotation |  3.27e-13  1.47e-12  1.21e-06  1.21e-06 |     0      8      0
leg_L  (Y-down, ctrl YZX)                    spin360   position |  2.89e-13  1.24e-12  0.00e+00  1.21e-06 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    spin360   rotation |  2.89e-13  1.24e-12  0.00e+00  1.21e-06 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    straight  position |  3.62e-13  3.73e-06  1.71e-06  5.12e-06 |     0      0   2000
leg_L  (Y-down, ctrl YZX)                    straight  rotation |  3.62e-13  3.73e-06  1.71e-06  5.12e-06 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    offHinge  position |  3.69e-13  1.44e-12  1.21e-06  1.72e+00 |     0      0      0
leg_L  (Y-down, ctrl YZX)                    offHinge  rotation |  3.69e-13  6.49e-02  1.21e-06  1.16e-01 |     0      0      0
leg_R  (-Y-down, ctrl is joint, XZY)         normal    position |  5.85e-13  2.99e-12  1.21e-06  1.71e-06 |     0      1      0
leg_R  (-Y-down, ctrl is joint, XZY)         normal    rotation |  5.85e-13  2.99e-12  1.21e-06  1.71e-06 |     0      1      0
leg_R  (-Y-down, ctrl is joint, XZY)         fast      position |  5.80e-13  2.28e-12  1.71e-06  1.21e-06 |     0     11      0
leg_R  (-Y-down, ctrl is joint, XZY)         fast      rotation |  5.80e-13  2.28e-12  1.71e-06  1.21e-06 |     0     11      0
leg_R  (-Y-down, ctrl is joint, XZY)         spin360   position |  6.00e-13  2.44e-12  1.71e-06  1.21e-06 |     0      0      0
leg_R  (-Y-down, ctrl is joint, XZY)         spin360   rotation |  6.00e-13  2.44e-12  1.71e-06  1.21e-06 |     0      0      0
leg_R  (-Y-down, ctrl is joint, XZY)         straight  position |  5.23e-13  3.48e-06  1.71e-06  4.68e-06 |     0      0   2000
leg_R  (-Y-down, ctrl is joint, XZY)         straight  rotation |  5.23e-13  3.48e-06  1.71e-06  4.68e-06 |     0      0      0
leg_R  (-Y-down, ctrl is joint, XZY)         offHinge  position |  5.76e-13  3.54e-12  1.21e-06  1.72e+00 |     0      0      0
leg_R  (-Y-down, ctrl is joint, XZY)         offHinge  rotation |  5.76e-13  6.49e-02  1.21e-06  1.16e-01 |     0      0      0

[rig] sampling interval (arm_L, normal): errors at the sampled frames
  step 0.5 : wrist 3.48e-13  elbow 1.07e-12
  step 2.0 : wrist 3.09e-13  elbow 6.98e-13
  step 5.0 : wrist 2.73e-13  elbow 9.55e-13

[rig] optional key reduction (linear tangents), arm_R / normal, 2000 frames
  tol 0.001 (units / deg): keys 18000 -> 17127 (95%), max wrist error 0.0013 units
  tol 0.010 (units / deg): keys 18000 -> 13254 (74%), max wrist error 0.0193 units
  tol 0.050 (units / deg): keys 18000 -> 9441 (52%), max wrist error 0.1094 units

[perf] engine cost only (matching + channel solving + synthetic rig evaluation)
   10000 frames, 1 limb : 0.031 s  (3.09 us / frame)  max wrist err 2.8e-12, elbow err 1.1e-11, flips 0
  100000 frames, 1 limb : 0.319 s  (3.19 us / frame)  max wrist err 3.7e-11, elbow err 1.4e-10, flips 0
  match+solve only: 1.451 us per limb-sample  (100k frames x 4 limbs = 0.58 s)  [1]

ALL CHECKS PASSED (0 failed checks)
```
