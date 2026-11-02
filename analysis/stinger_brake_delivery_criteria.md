# Stinger brake-delivery compensation: acceptance criteria (fixed 2026-10-30, before any fit or evaluation)

Data: openpilot-long drives 00000003--0f3f0eb0f2 (t3, build 59cdae0a69) and 00000004--a5fd1905e8 (t4, build bf5b64ef18).
Sent accel equals the planner request (|out-req| > 0.05 on 1% of pid samples), so the request is the plant input.
Plant: shortfall sf(req, v) = aEgo - req over 0.5 s windows with a steady request (range < 0.3), 0.2 s actuation lag,
engaged, no pedals/override/standstill, control state pid or stopping. Positive = the car brakes less than asked.
Model: sf = s(v) * g(req), fitted on one drive, tested on the other (both directions).

Pass/fail (held-out drive; residual = sf_obs - sf_fit, which is exactly the shortfall left after inverting the model):
- A1: windows with v 1.0-4.5 m/s and req -1.0..-0.45: |median residual| <= 0.10 m/s^2 (before: about 0.25).
      Every (v, req) cell with n >= 30 (>= 15 for the sparse drive t3): |median residual| <= 0.15.
- A2: untouched regimes (v >= 6 m/s, req > -0.2, req <= -1.5 with v >= 4.5): policy adds < 0.05 m/s^2.
- A3: added decel is never negative (never brakes less than asked), <= 0.35 m/s^2 everywhere, output >= ACCEL_MIN, and on the logged
      requests the output's sample-to-sample step exceeds the request's own step by <= 0.05 m/s^2 per 50 ms.
- A4: unit tests: identity when not longActive, with override, req >= -0.2, or CarParams not the Stinger.
- A5 (descriptive only): predicted added decel and distance saved on the 11 driver-braked takeovers (10-28, 10-29).
Not claimed: the transient accel->brake lag (n=5 ramps at 2-4 m/s, median 0.95 s vs 0.5 s elsewhere; too thin to act on).
