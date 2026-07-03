# Closed-loop experiment videos

Short recordings of the single-agent distance-regulation experiments referenced
in the mobile-dashboard paper. Clips are re-encoded (H.264, CRF 28, max width
1280 px) to keep file sizes small.

| File | Shows | Firmware | Setup |
|------|-------|----------|-------|
| `single_train_demo_2026-07-03.mp4` | Single-train closed-loop distance regulation, current build | 2.4.1 | Untethered (matches the paper's reported runs) |
| `dashboard_live_2026-07-03.mp4` | The web dashboard plotting a live PID step (18→28 cm) in real time, with a mid-run setpoint change to 22 cm; driven entirely through the page UI over the train's WebSocket | 2.4.1 | Untethered; screen capture of the dashboard served by the train at 192.168.4.1 |
| `zn_gain_progression.mp4` | Ziegler–Nichols proportional-gain progression toward marginal stability | 2.3 | USB-tethered |
| `pid_step.mp4` | Full PID controller tracking a setpoint step | 2.3 | USB-tethered |

**Note on firmware.** The `zn_gain_progression` and `pid_step` clips were
recorded on firmware 2.3 with the agent USB-tethered. That earlier configuration
runs the control loop more slowly (~95 ms vs. the current 50 ms) and the cable
adds drag, so the ultimate gain observed in those clips is lower than the
$K_u \in [64, 80]$ range reported in the paper (which comes from firmware 2.4.1,
untethered). They are included to illustrate the qualitative phenomena — the loop
crossing from well-damped toward marginal stability, and PID step tracking — not
the exact numerical values in the text.
