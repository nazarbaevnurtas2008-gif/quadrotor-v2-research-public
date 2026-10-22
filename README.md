# Nonlinear Quadrotor Control — Reconstructed V2 Research Evidence

**Nurtas Nazarbayev · NIS Shymkent Abay · Kazakhstan · 2026**

**Status:** Unpublished technical manuscript and software-in-the-loop study (not peer reviewed, not hardware-validated). The numerical plant is a **reconstruction** from a documented methodology, not recovered original source code.

## Research question

Does performance-oriented and robustness-oriented automatic PID tuning outperform a carefully tuned conventional PID baseline and an integral LQR controller under common reconstructed quadrotor uncertainties?

## Frozen paired evaluation

100 final conditions × 4 controllers = 400 paired simulated runs. Altitude-task pass rates were:

| Controller | Passed/100 |
| --- | ---: |
| Carefully tuned conventional PID | 95 |
| Performance Auto-PID | 95 |
| Robust Auto-PID | 93 |
| Integral LQR | 99 |

None of the adjusted paired pass-rate comparisons against the conventional PID baseline established a statistically significant advantage. The LQR controller also exhibited weaker roll/yaw behavior; lateral x/y/yaw are not closed-loop stabilized. **No full-flight stability or flight-safety conclusion is made.**

## Key files

- `Quadrotor_V2_Manuscript_Technically_Checked.pdf`: complete research manuscript.
- `frozen_evidence/final_400_paired_controller_trials.csv`: all final-run metrics.
- `frozen_evidence/final_conditions_FROZEN.csv`: conditions and sampling metadata.
- `frozen_evidence/frozen_source/`: actual reconstruction controller and plant code.
- `frozen_evidence/FROZEN_FINAL_PROTOCOL.json`, `frozen_evidence/PRE_EXECUTION_SEED_ERRATUM.md`: protocol and disclosed pre-execution seed correction.
- `independent_trajectory_replay.py`, `replay_verification.json`, `replayed_max_altitude_all_400.csv`: replay evidence.
- `Technical_Review_Report_KK.md`: Kazakh-language methodological limitations.
- `verify_public_release.py`: lightweight standard-library checks for included frozen CSV.

## Validation and limits

The final conditions have already been inspected; any new controller tuning must use a **different** evaluation plan and cannot reuse these conditions as blind. Several plant parameters (including aerodynamic coefficients and sensor assumptions) are reconstruction choices. This repository is a computational **research artifact**, not peer-reviewed validation or experimental motor calibration. A third-party critical technical review is pending.

## Authorship, AI and scientific integrity

Nurtas Nazarbayev reports that the initial research idea was his; Python development involved AI assistance; and he primarily analyzed results and wrote the manuscript. AI also contributed to code, simulation execution, interpretation, scientific drafting and technical revisions. These are self-reported roles, not independently audited percentages or a claim of unaided execution. See **[CONTRIBUTIONS_AND_AI.md](CONTRIBUTIONS_AND_AI.md)** for the detailed disclosure.

Dr. Ali Nasir offered preliminary methodological feedback but is not a supervisor, coauthor or peer reviewer. The student must personally validate what he submits to academic institutions or science competitions.

**Research integrity:** [CI workflow](.github/workflows/research-integrity.yml), [reproducibility checksums](SHA256SUMS.txt), [scope and attribution](CONTRIBUTIONS_AND_AI.md). CI consistency checks are not evidence of physical flight safety.

## Archive, DOI and citation

[![Zenodo DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23562460.svg)](https://doi.org/10.5281/zenodo.23562460)

The **public GitHub release** [`v2.0.0-research-artifact`](https://github.com/nazarbaevnurtas2008-gif/quadrotor-v2-research-public/releases/tag/v2.0.0-research-artifact) was published on 22 October 2026. Zenodo's GitHub integration displayed the DOI **[10.5281/zenodo.23562460](https://doi.org/10.5281/zenodo.23562460)** for this repository on the same date. See [`CITATION.cff`](CITATION.cff) for the citation metadata.

**Citation status:** This is a Zenodo-archived, **unpublished simulation research artifact**, not a peer-reviewed journal article. The DOI does not imply scientific endorsement, flight validation, or independent review. The release contains a reconstructed model, not original recovered plant source code.

Code is offered under MIT only to the extent of the author's rights, while the author's original manuscript text, figures, and result tables are offered under CC BY 4.0. See [`LICENSE`](LICENSE) and [`LICENSE-CONTENT.md`](LICENSE-CONTENT.md) for scope, third-party exclusions, and AI-assisted material limitations.

## Publication status

This public repository was assembled from a clean snapshot without importing the older private repository's Git history. The DOI metadata in this **main branch** was added **after** the tagged `v2.0.0-research-artifact` release; the tagged archive remains a frozen snapshot. Researcher contributions and AI assistance are disclosed in [`CONTRIBUTIONS_AND_AI.md`](CONTRIBUTIONS_AND_AI.md). GitHub Actions verifies internal consistency, not independent scientific correctness or flight safety.
