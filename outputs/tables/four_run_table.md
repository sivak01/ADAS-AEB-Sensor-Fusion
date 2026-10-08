# Four-run comparison

| run | observability_variant | n_scored | correct_action_rate | false_brake_rate | missed_brake_rate | under_brake_count | over_brake_count | phantom_brake_count | false_brake_incl_phantom_rate |
|---|---|---|---|---|---|---|---|---|---|
| lidar | all | 133 | 97.0% (129/133) [0.93, 0.99] | 0.8% (1/126) [0.00, 0.04] | 28.6% (2/7) [0.08, 0.64] (indicative only) | 1 | 0 | 1/71 | 1.0% (2/197) [0.00, 0.04] |
| lidar | obs | 133 | 97.0% (129/133) [0.93, 0.99] | 0.8% (1/126) [0.00, 0.04] | 28.6% (2/7) [0.08, 0.64] (indicative only) | 1 | 0 | 1/71 | 1.0% (2/197) [0.00, 0.04] |
| radar | all | 133 | 94.0% (125/133) [0.89, 0.97] | 3.2% (4/126) [0.01, 0.08] | 42.9% (3/7) [0.16, 0.75] (indicative only) | 1 | 0 | 0/71 | 2.0% (4/197) [0.01, 0.05] |
| radar | obs | 133 | 93.2% (124/133) [0.88, 0.96] | 4.7% (6/129) [0.02, 0.10] | 50.0% (2/4) [0.15, 0.85] (indicative only) | 1 | 0 | 0/71 | 3.0% (6/200) [0.01, 0.06] |
| camera | all | 133 | 95.5% (127/133) [0.91, 0.98] | 0.8% (1/126) [0.00, 0.04] | 71.4% (5/7) [0.36, 0.92] (indicative only) | 0 | 0 | 1/71 | 1.0% (2/197) [0.00, 0.04] |
| camera | obs | 133 | 95.5% (127/133) [0.91, 0.98] | 0.8% (1/126) [0.00, 0.04] | 71.4% (5/7) [0.36, 0.92] (indicative only) | 0 | 0 | 1/71 | 1.0% (2/197) [0.00, 0.04] |
| fused | all | 133 | 87.2% (116/133) [0.80, 0.92] | 11.1% (14/126) [0.07, 0.18] | 28.6% (2/7) [0.08, 0.64] (indicative only) | 1 | 0 | 1/71 | 7.6% (15/197) [0.05, 0.12] |
| fused | obs | 133 | 87.2% (116/133) [0.80, 0.92] | 11.1% (14/126) [0.07, 0.18] | 28.6% (2/7) [0.08, 0.64] (indicative only) | 1 | 0 | 1/71 | 7.6% (15/197) [0.05, 0.12] |

## Footnotes

- **1_radar_doppler**: Radar-only ran with Doppler DISABLED (G17) -- this reflects this configuration, not radar's real capability (what-not-to-do.md §7).
- **2_r_measured_vs_assumed**: Per-sensor measurement noise (R) status: camera=PLACEHOLDER, lidar=ASSUMED, radar=ASSUMED
- **3_indicative_only**: 1 row(s) fall under MIN_COUNT_FOR_RATE and are labelled 'indicative only': ['missed_brake_rate (all 4 streams -- eval split has only 7 danger events)']
- **4_class_policy**: Class recorded (Detection.cls) but never used for gating in any of the 4 runs (DEC-2).
- **5_split**: Split used: eval. Tuning was performed on the tuning split only.
- **6_distance_definition**: Distance definition (DEC-1): nearest_surface_to_ego.
- **7_gt_events**: 315 total GT rows, 11 danger events (3.5% of GT rows) -- nuScenes-mini has few danger events; treat small-denominator rates as indicative only.