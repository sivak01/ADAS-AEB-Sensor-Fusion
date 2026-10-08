# Deployment architecture reference — from prototype to on-vehicle function

**Source:** built from "System Architect Role – Deploying the Multi-Sensor TTC Fusion
Pipeline" (PDF, dated 2026-09-20), which gap-analysed **V1's** pipeline specifically — its
cited numbers (404 samples, fused MAE 3.80s vs. single-sensor, 71% pass-through, 60s TTC
cap, 27% GT match, run-to-run non-determinism, fixed-height monocular depth, DBSCAN/
RANSAC tuning) are all V1's Step 6 results, not this project's.

**Automation-level adjustment (the one change that reshapes everything else below):**
the source document assumes **SAE Level 1–2** (driver-supervised — a human is always the
fallback). This project's actual deployment target is **SAE Level 2 through Level 4 and
above**. Sections below are marked **UNCHANGED** where the source's content holds
regardless of level, and **ADAPTED** where crossing from L2 into L3/L4 changes the
requirement, with the reasoning spelled out rather than just the new conclusion.

**How to use this file:** background/roadmap reference for deployment thinking, not a
build-order step. The active project (`README_MASTER.md`) is still an offline
nuScenes-mini research prototype — nothing here changes step00–step11. Consult this file
when discussing deployment strategy, writing the portfolio narrative, or if/when a real
deployment phase actually starts.

---

## 1. Why L1–2 vs. L2–4+ is not a small adjustment — ADAPTED

SAE levels 1–4 don't scale up smoothly; there is a hard break at the boundary between L2
and L3 in *who is responsible for catching a failure*:

| | Fallback model | What that means for this pipeline |
|---|---|---|
| **L1–L2** (source doc's assumption) | Human is always the fallback. The system assists/warns; a human is expected to be monitoring and can always intervene. | A missed detection or a wrong TTC is a **secondary** risk — the driver is the last line of defense. Standard **fail-safe** design: on fault, disable/reduce the function and hand back. |
| **L3** | System is the primary controller inside a defined ODD; human is fallback **on request**, with a bounded transition time (commonly ~10s, per UN R157/ALKS-style regulation). | A fault must trigger a **timely transition demand**, and the system must bridge safely until the human actually retakes control (or is deemed non-responsive). |
| **L4 (and L5 aspirational)** | System must reach a **Minimal Risk Condition (MRC)** entirely on its own, inside its ODD — no human fallback assumed at all. | A missed/false TTC is now a **primary** causal hazard, not a secondary one a human backs up. Requires **fail-operational** design: the system must survive a fault, or autonomously execute a Minimal Risk Maneuver (MRM, e.g. controlled stop in lane / pull-over) — never just "hand back and hope." |

**Practical consequence for this program:** treat **L2 (warning / AEB-assist)** as the
realistic near-term target (this is what the source document's whole gap analysis was
actually written for), and **L3/L4 as the stated long-term direction the architecture
should not foreclose** — i.e. design redundancy, interfaces, and the safety monitor so
they don't have to be rebuilt from scratch later, without pulling the full L4 safety-case
scope into an early-phase research prototype. Jumping straight to L4-grade requirements
on a nuScenes-mini prototype would be solving a program-cost problem the project doesn't
have yet.

---

## 2. Gap analysis: prototype vs. deployment — UNCHANGED (holds at any level)

These are properties of going from *offline batch on recorded data* to *any real-time
on-vehicle function*, independent of automation level:

| Area | Prototype (V1, as evaluated) | Gap to real deployment |
|---|---|---|
| Execution | Batch notebooks, one step after another | Streaming, multi-threaded runtime, bounded latency |
| Frame rate | 2 Hz keyframes (V1 read `samples/` only) | Sensor-rate operation, typically 10–20 Hz |
| Data | 404 samples, 10 scenes, one region | Diverse ODD: weather, night, geography, traffic |
| Repeatability | Run-to-run differences (e.g. 2,898 vs. 2,838 LiDAR tracks) | Deterministic, versioned, reproducible outputs |
| Paths/config | Hard-coded paths, files passed between notebooks | Configuration management, message-based interfaces |
| Safety | None: no fault handling, no fallback, no monitoring | Full safety concept, diagnostics, degraded modes |

**Already closed by this project's design, not a future fix:** the frame-rate row. G3 /
`nuscenes-reference.md` §6 / `lessons-from-v1-v2.md` item E all mandate reading `sweeps/`
at native rate from the start. Step 00's real preflight against `archive/` measured this
project's actual rates — **LiDAR ≈20.1 Hz, radars ≈13.3–13.4 Hz, cameras ≈10 Hz** —
already inside the 10–20 Hz band the source document names as the deployment target.

---

## 3. Fusion-benefit gap — UNCHANGED, and the first gate at any level

The source document's central point stands regardless of automation level: **until fused
TTC beats the best single sensor, added fusion complexity is hard to justify.** Its named
likely causes (single-sensor pass-through rate, fragmented radar tracks, an unbounded TTC
cap) map directly onto this project's own G13 ("if fusion does not beat the best single
sensor, say so plainly") and its four-run mandatory comparison (lidar/radar/camera/fused).
This project's `step11_runs_tuning_report.md` is the step that actually answers this
question for this codebase — nothing here changes what that step must do.

---

## 4. Target system architecture — ADAPTED

The source document's streaming graph (sensors → time sync/ego-motion → per-sensor
perception → per-sensor tracking → fusion → TTC → decision, with a safety monitor beside
the main path) is a reasonable skeleton at any level. Two additions are needed once L3/L4
is a stated target, not proposed by the source document at all because it assumed L1–2:

- **ODD monitor.** A dedicated component that continuously verifies operation is inside
  the approved Operational Design Domain (speed range, road type, weather, geofence).
  At L1–2 this is informal (the driver notices). At L3 it must trigger a transition
  demand before an ODD boundary is crossed; at L4 it must trigger an MRM. This is a new
  block, not a relabeling of the existing safety monitor.
- **Minimal Risk Maneuver (MRM) path.** A parallel actuation path from the safety monitor
  that can independently bring the vehicle to a stop/safe state without depending on the
  main decision layer being healthy. Not needed at L1–2 (hand back to driver is the MRM);
  mandatory at L4.

---

## 5. Core responsibilities by workstream — mostly UNCHANGED, two ADAPTED

The source document's seven workstreams (requirements/ODD, architecture/interfaces,
sensor suite/calibration, compute/platform, safety/cybersecurity, validation, integration/
lifecycle) still partition the work correctly at any level. What changes:

- **Workstream 1 (Requirements and ODD) — ADAPTED.** At L3/L4 the ODD is a formal,
  legally load-bearing artifact (specific roads/speeds/weather the system is certified
  for), not just a design input — because inside that ODD, the system *is* the driver.
- **Workstream 5 (Safety) — ADAPTED.** Must explicitly add fail-operational analysis and
  an MRM strategy once L3/L4 is in scope, on top of the source document's fail-safe/
  fault-detection framing (which was sufficient for L1–2 alone).

---

## 6. Real-time, compute and hardware — mostly UNCHANGED, ADAPTED on redundancy

The per-stage latency budgets, hardware-acceleration needs (port hot paths to C++/CUDA,
quantise YOLO, bound worst-case points/clusters/tracks per frame) and the ~100ms
end-to-end target are sound at any level — these are physics/engineering constraints, not
policy choices.

**ADAPTED:** at L1–2, single-string compute is common practice (the human is the
system's redundancy). At L3, and especially L4, **redundant/diverse compute and sensing**
becomes a hard requirement, not an option — because there is no human fallback to lean
on if one compute path fails silently. This has a direct architectural consequence: the
sensor suite and compute platform decisions in workstream 3/4 should be re-run assuming
at least one sensor modality can independently trigger an emergency stop even if the
fusion/main path is unavailable.

---

## 7. Standards — ADAPTED (this is where the level change bites hardest)

| Standard | What it covers | L1–2 relevance (source doc) | L3–L4+ relevance (adjusted) |
|---|---|---|---|
| ISO 26262 | Functional safety of E/E systems | Hazard analysis, ASIL rating, safety goals | Same, but ASIL determination shifts **higher** — perception/TTC becomes a *primary* safety-critical path (likely ASIL C/D), not one a human backs up |
| ISO 21448 (SOTIF) | Safety when design is correct but performance is limited | Central for perception: missed detections, degraded conditions | **Even more central** — the system must self-certify perception sufficiency with no human catching residual risk |
| ISO/PAS 8800 | Safety of AI in road vehicles | Covers YOLO and learned components | Same coverage, higher weight — decisions made with no human check |
| ISO/SAE 21434, UN R155 | Cybersecurity engineering/management | Sensor spoofing, secure messaging | Same attack surface, **worse consequence** of compromise without human oversight |
| UN R156 | Software update management | OTA updates of models/parameters | Same |
| UN R152 / Euro NCAP AEB | Regulatory/rating tests for AEB | Directly applicable (this is an L1–2 function) | Applicable only to the L2 AEB-assist mode of this program, not L3/L4 |
| **UN R157 (ALKS)** *(add)* | Automated Lane Keeping Systems | Not in source (L1–2 doesn't need it) | **The actual L3 regulatory framework** — transition-demand timing, minimum risk maneuver requirements |
| **UL 4600** *(add)* | Safety case framework for autonomous products | Not in source | **Primary safety-case standard for L3–L5** — goal-based safety case, not ISO 26262's process-based approach alone |
| **ISO 34502 / 34503** *(add)* | Scenario-based safety evaluation for automated driving | Not in source | Needed for L3+ scenario-coverage arguments |
| **SAE J3016 / J3131** *(add)* | Defines the levels themselves / MRM terminology | Not in source | Reference framework for stating what level is claimed and what an MRM is |
| Automotive SPICE | Software process assessment | Traceable requirements/design/test/review | Same at any level |

L4 currently has **no single harmonized global regulation** (unlike L3's UN R157) —
track jurisdiction-specific frameworks (US NHTSA/state DMV disengagement-reporting
regimes, China's pilot regulations, the EU's emerging framework) per target market.

---

## 8. Safety design decisions — ADAPTED

| Decision | L1–2 framing (source doc) | L3–L4 framing |
|---|---|---|
| Fault detection | Detect fault, reduce/disable function, hand back | Detected fault must trigger a **bounded-time transition demand** (L3) or an **autonomous MRM** (L4) — "hand back" is not an option at L4 |
| Degraded modes | Reduce confidence / warn less when a sensor is lost | Losing a sensor in the *minimum required redundant set* must be treated as an **ODD-exit trigger**, not just reduced confidence |
| Redundancy/plausibility | Cross-check TTC between sensors, pass disagreement as uncertainty | Becomes **mandatory** cross-validation the vehicle uses to decide, autonomously, whether it still trusts the ODD it's operating in |
| Decision rules | Hysteresis/persistence/confidence gating to avoid acting on one noisy value | Same principle, but thresholds must be validated against an **MRM-triggering** framework, not just a warning framework |

---

## 9. Validation — UNCHANGED structure, ADAPTED acceptance criteria and scale

The four-level validation ladder (software-in-the-loop → simulation → closed track →
public road shadow mode) holds at any automation level. What changes is what "pass"
means and how much of it is needed:

- **L1–2 headline metric:** false-warning rate per 1,000 km (as in the source document).
- **L3–L4 additional metrics:** disengagement rate (interventions per mile, in the style
  of California DMV disengagement reporting), MRM success rate, ODD-boundary-detection
  accuracy, transition-demand timing accuracy.
- **Scale:** L4 public-road validation conventionally requires either safety-driver
  miles at a scale several orders of magnitude beyond an L2 AEB program, or heavy
  simulation to substitute for real miles (a widely cited RAND Corporation estimate puts
  naive real-world L4 validation in the hundreds of millions to billions of miles) — this
  has major program-cost implications the source document's L1–2 framing never had to
  raise, and is worth stating explicitly whenever L3/L4 timelines are discussed.

---

## 10. Data, calibration and model lifecycle — mostly UNCHANGED, one addition for L4

Training-data ODD coverage, detector fine-tuning (replacing V1's fixed monocular-depth
class heights), parameter re-tuning under configuration control, calibration management,
moving to reference-instrumented ground truth, deterministic/versioned pipelines, and
fleet monitoring all apply regardless of level.

**Addition for L4:** many production L4 stacks depend on HD maps + tightly-bounded
GNSS/INS localization, not live perception alone. Whether this program goes map-reliant
("L4 highway pilot" style) or stays map-light is itself an open architecture decision that
doesn't arise at L1–2, and should be named explicitly rather than defaulted into.

---

## 11. Roadmap, risks and open decisions — ADAPTED

The source document's 4-phase roadmap (Foundation → Real-time port → Vehicle integration
→ Validation/release) and its Phase-1 exit gate ("fused TTC beats best single sensor on a
larger dataset") both still hold — that gate is level-independent and should stay the
Phase-1 target regardless of the L2–L4 ambition.

**Reframe the source document's open decision "Function level: warning only, or warning
plus automatic braking"** as: which of **L2 / L3 / L4** is the committed near-term target
vs. the stated long-term direction. Recommended framing for this program: **build for L2
now** (this is what the fusion-benefit gate, the latency budget, and the current
validation plan are actually sized for), **architect so L3/L4 isn't foreclosed** (ODD
monitor hook, MRM path stub, redundant-sensing-capable interfaces), and **do not pull
L3/L4-grade safety-case scope (UL 4600, disengagement-scale validation) into the current
phase** — that scope only activates if/when the program actually commits to L3/L4.

Risk table from the source document, unchanged in substance, with one addition:

| Risk | Why it matters | Mitigation |
|---|---|---|
| Fusion does not improve accuracy | Undermines the multi-sensor business case | Diagnose track fragmentation and single-sensor pass-through first (this project's step11) |
| Latency exceeds budget | Late warnings/actions reduce safety value | Early profiling on target hardware, hard caps on load |
| Perception failure in unseen conditions | Missed or false TTC | SOTIF analysis, target-domain data, shadow mode |
| Calibration drift | Corrupts projection and fusion | Runtime plausibility checks, recalibration procedure |
| Radar track fragmentation | Weak radar contribution | Improve association and stationary-object handling |
| Scope creep to higher automation | Raises safety level and cost sharply | Freeze the *committed* level (L2) at Phase 1; keep L3/L4 as an explicit, separately-gated future decision, not an implicit drift |
| **(added) Committing to L3/L4 safety-case scope too early** | UL 4600 / disengagement-scale validation is a different order of program cost than an L2 AEB assist | Keep L3/L4 architecture-*compatible*, not architecture-*committed*, until the level decision is made explicitly |
