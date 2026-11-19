# FkToIkBake — native C++ FK → IK bake for Maya 2027

One job: convert FK limb animation into IK control animation, bake it, and report how accurate the result is.

## Status

| Part | State |
|---|---|
| Matching & baking engine (`engine/`) | Tested with no Maya involved: 40 rig x animation x mode cases plus unit tests, all passing (`docs/TEST_REPORT.md`). |
| `fkToIkBake` command (`maya/`) | Built with Visual Studio 2026 against Maya 2027 (0 warnings). **Tested in Maya 2027** on an FK/IK arm under a moving, spinning root (`tests/maya/maya_test_report.txt`): wrist ~0.0005 units, elbow ~0.0007 units, wrist rotation 0 deg, in both modes; undo/redo; 5000 frames bake in about 3 s. |
| Drag-and-drop installer | Run in Maya 2027. |
| Bake window (`fkToIkBakeUI`) | Written for Advanced Skeleton naming. **Not yet run in Maya.** |
| Advanced Skeleton names | Taken from AS's standard naming; not yet checked on a real AS character. |

**Download:** the ready-to-install package (with the compiled `.mll`) is attached to the GitHub Release. Unzip it and drag `install_fkToIkBake.mel` into the Maya 2027 viewport.

## Layout

```
engine/      Maya-independent engine (C++17): math, matching, channel solve, Euler filter, key reduction, rig maps
maya/        fkToIkBake MPxCommand: flag parsing, MDGContext sampling, undoable key writing, in-Maya validation
tests/       synthetic rig + independent Rotate-Plane IK solver + validation suite
package/FkToIkBake/   what users install: install_fkToIkBake.mel, scripts/, maps/, plug-ins/win64/2027/
CMakeLists.txt, build_windows.bat
```

## How the engine works

For each limb the engine **calibrates once** on the IK side of the rig, at the calibration frame:

* `ctrlOffset = ikCtrlWorld · ikEndJointWorld⁻¹`. This records how the IK control sits relative to the IK wrist/ankle joint, which covers its orient-constraint offset, the handle placement, any pivot offset and the control's own orientation.
* The pole's angle around the shoulder→wrist axis, relative to the chain plane. This absorbs any ikHandle `twist` and any off-plane pole placement.
* The chain-plane normal expressed in the IK elbow/knee frame (used by rotation mode).
* The rig's pole distance.

Then it runs **one pass over time for all limbs**, because each Maya evaluation is the expensive part. At each sample:

1. The FK joint world matrices are read with an `MDGContext`, which means no timeline change and no viewport refresh.
2. **IK control target** = `ctrlOffset · fkEndWorld`. This matches wrist/ankle position and orientation exactly.
3. **Pole target**:
   * **Position mode**: the pole sits in the plane of the FK shoulder/elbow/wrist, on the elbow side, rotated by the calibrated angle. This matches elbow/knee *position* exactly. Within about 1.5° (arms) or 2° (legs) of fully straight, the bend plane is ill-defined, so the pole blends smoothly to the rotation-derived direction instead of flipping.
   * **Rotation mode** (like Advanced Skeleton's *MotionCapture* option): the bend plane comes from the FK elbow/knee *rotation*. It stays stable on straight limbs and mocap, and preserves elbow twist.
4. **Channel solve**: the target world matrix is converted into the control's own `translate`/`rotate` values. This is exact for parent spaces, every rotate order, `rotateAxis`, rotate/scale pivots and their translates, shear, non-unit scale, and joint controls (`jointOrient`, inverse parent scale).
5. **Euler continuity**: each rotation picks the equivalent Euler triple closest to the previous key. Both the direct and the alternate (x+180, 180−y, z+180) solutions are tried, with ±360° wraps on every axis. This handles crossing 180° and 360° and spins of any length.

After the IK controls are keyed, the pole parent matrices are read again and the pole keys are re-solved. Many rigs (including Advanced Skeleton's pole *follow*) parent the pole under a space that follows the IK control, which has just changed.

Keys: one key per sample is written with a single bulk `addKeys` call per channel. Constant channels collapse to two keys, which loses nothing. `-tolerance` enables Ramer–Douglas–Peucker reduction with linear tangents, so the per-channel error bound is guaranteed.

### Arm mode vs leg mode

Both run the same two-bone math, because an elbow and a knee are the same problem for a Rotate-Plane IK chain. The mode changes the defaults: legs start the straight-limb fallback slightly earlier, since legs spend more time almost straight. Legs also come with caveats about foot rigs (see Limitations). I've stated this plainly rather than inventing differences that don't exist.

## Rig compatibility: rig maps

Automatic detection across Advanced Skeleton, mGear and custom rigs is not reliable, so each rig is described once in a small INI file:

```ini
[L_arm]
type        = arm            ; arm | leg
mode        = position       ; position | rotation
fkUpper     = FKXShoulder_L  ; FK-driven JOINTS
fkMid       = FKXElbow_L
fkEnd       = FKXWrist_L
ikUpper     = IKXShoulder_L  ; IK-driven JOINTS
ikMid       = IKXElbow_L
ikEnd       = IKXWrist_L
ikControl   = IKArm_L
poleControl = PoleArm_L
switchAttr  = FKIKArm_L.FKIKBlend   ; optional
switchFk    = 0
switchIk    = 10
```

`package/FkToIkBake/maps/` contains `advancedSkeleton_biped.ini`, which uses the standard AS names. **Verify those names on your character.** It also contains `template.ini`, with every key documented. The names get a namespace prefix at run time, so one map works for any number of referenced characters.

Single-chain rigs, where FK and IK blend on the same joints, are supported. Map the same joints as `fk*` and `ik*` and give the switch. The tool then reads FK with the switch at `switchFk` and calibrates with it at `switchIk`. This only works if the switch is not animated or connected; otherwise the tool warns you.

## Usage

```mel
// all limbs in the map, playback range, with an error report
fkToIkBake -rigMap "D:/rigs/advancedSkeleton_biped.ini" -namespace "hero:" -validate;

// selected limbs, explicit range, sub-frame sampling, mocap mode
fkToIkBake -rigMap $map -limb "L_arm" -limb "R_arm" -startFrame 1 -endFrame 2400 -step 0.5 -mode rotation;

// no map file: one limb inline
fkToIkBake -fkJoints FK_sh FK_el FK_wr -ikJoints IK_sh IK_el IK_wr -ikControl armIK_ctrl -poleControl armPV_ctrl -type arm;
```

| Flag | Meaning |
|---|---|
| `-rigMap -mp` / `-limb -l` (multi) / `-namespace -ns` | rig map, which limbs to bake (default: all), name prefix |
| `-startFrame -sf` / `-endFrame -ef` / `-animationRange -ar` | range (default: playback range) |
| `-step -st` | sampling interval in frames (default 1; the last frame is always sampled) |
| `-mode -mo` | override the map's mode: `position` or `rotation` |
| `-tolerance -tol` | key reduction tolerance (scene units and degrees); 0 = off (default) |
| `-switchToIk -sik` | set or key the switch to IK after the bake (default true) |
| `-calibrationFrame -cf` | frame used to calibrate the IK side (default: start). Pick a frame where the IK limb is bent. |
| `-poleDistance -pd` | override the pole distance |
| `-evalMode -ev` | `context` (default, fast) or `timeline` (moves the time slider; for rigs that only evaluate correctly at the current time) |
| `-validate -val` | after baking, re-evaluate the IK joints at every sample and report max/mean position error and wrist/ankle rotation error against the original FK |

The command is undoable (one undo removes every key it wrote and every curve it created) and can be cancelled with Esc. It prints per-limb diagnostics (max reach, over-extended frames, straight-limb fallback frames) and a timing breakdown (prepare / sample / solve / write / validate).

The shelf button `FK>IK` opens the **Bake window** (`fkToIkBakeUI`): pick the character namespace and `FK to IK : Arm_L / Arm_R / Leg_L / Leg_R` (or All limbs), tick *baking MotionCapture* for rotation mode, and press **Ok, Bake**. It uses Advanced Skeleton names, so no rig map is needed.

## Building the .mll (one time, on Windows)

1. Install **Visual Studio 2022** (Desktop development with C++) and **CMake 3.20+**. Check Autodesk's Maya 2027 devkit notes for the exact required toolset.
2. Download the **Maya 2027 devkit** from the Autodesk developer site and unzip it, e.g. to `C:\devkitBase`.
3. Run `build_windows.bat "C:\devkitBase"`. This builds the plugin and runs the engine tests, and places `fkToIkBake.mll` in `package\FkToIkBake\plug-ins\win64\2027\`.
4. Zip `package\FkToIkBake` and hand it to users.

## Installing (users)

Drag `install_fkToIkBake.mel` into the Maya 2027 viewport. It copies the package to `Documents/maya/modules/FkToIkBake`, writes a `.mod` file, loads the plugin, sets it to auto-load and adds the shelf button. No environment variables or developer tools are needed.

## Limitations (real, not hidden)

* **The FK limb must be planar for an exact elbow/knee match.** A Rotate-Plane IK chain can only bend on its hinge. If the animator also rotated the FK elbow off its hinge axis, position mode still matches wrist, elbow and shoulder *positions* exactly, but the elbow *twist* can't be reproduced. Rotation mode keeps the twist closer and gives up some elbow position. The tests measure both (see the report).
* **Stretch.** If the FK limb is longer than the IK chain (FK stretch, or IK stretch off), IK can't reach. Those frames are counted and reported as `overExtended`.
* **Reverse-foot / roll attributes** on the IK foot (roll, heel, toe, tilt) and animated ikHandle twist must stay at their calibration values during the bake. They're folded into the calibration.
* **Locked channels, constraints and animation layers** on the IK or pole control are refused with a clear error, rather than being keyed underneath something that overrides them.
* Control scale, pivots and joint orient are read once and assumed not to be animated.
* `-step > 1` is exact at the sampled frames. In-between frames follow curve interpolation.
* Large Euler channel moves remain possible near gimbal lock (for example, 25–30° per frame on several axes). The filter always picks the mathematically closest Euler solution, and the test-suite checks this against brute force.
* `context` evaluation is the fast path. Rigs with nodes that only evaluate correctly at the current time need `-evalMode timeline`.

## In-Maya acceptance checklist (to do once the .mll is built)

1. Load the plugin and run the AS map on a character: `fkToIkBake -rigMap ... -validate`. Expect end/mid errors of roughly 1e-4 units or less.
2. Repeat with `-mode rotation` on a mocap clip with near-straight arms.
3. Undo, then redo.
4. Run on a referenced character with `-namespace`.
5. Bake a 10k-frame clip and note the timing line. In Maya, `sample` should dominate, since the engine solve is about 1.5 µs per limb-sample.
