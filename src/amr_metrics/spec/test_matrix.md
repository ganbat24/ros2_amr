# amr_metrics — Test Matrix

## Build-Time Tests

| Test | Expected Behavior |
|------|-------------------|
| `test_plot_metrics.py` | `align_odom` recovers a known rigid transform; plotting helpers handle empty and single-row inputs |
| Console scripts install | Wrappers land in `install/amr_metrics/lib/amr_metrics/`, not the source tree |
| ament linters | flake8 / pep257 clean |

## Runtime Tests (require a live stack)

| Test | Expected Behavior |
|------|-------------------|
| `ready_gate` | Returns success only once `/odom` moves > 0.03 m under a commanded probe |
| `record_trajectory` | Produces a CSV with monotonically increasing sim-time stamps |
| `run_validation` | Dispatches each goal, cancels a timed-out goal server-side, settles between goals |
| `plot_metrics` | Renders a PNG from a recorded CSV without a display |

## Known Gaps

- No automated test asserts tour *success rate*; that is a measured property
  of the whole stack, not a unit under test, and it is environment-dependent.
- Runtime tests are not wired into `colcon test` — they need a running
  simulation, which CI does not provide.
