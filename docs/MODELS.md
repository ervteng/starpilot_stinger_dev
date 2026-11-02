# Model Guide & T3 Delegation Matrix for OpenPilot / StarPilot

This guide outlines model strengths and delegation strategies across T3 Code providers (**Claude Code** and **Google Antigravity**) when working on this workspace (`/home/openpilotstuff`).

Model names and strengths below are not verified. Check them against `orchestrator_capabilities` before the first `delegate_task` in a thread.

---

## Model Strengths & Selection Matrix

| Model | Provider | Context & Latency | Best Suited For in this Workspace | When to Avoid |
| :--- | :--- | :--- | :--- | :--- |
| **Claude Opus 5.5**<br>`claude-opus-5-5` | Claude / Antigravity | Standard context<br>Medium latency | **Lead Architect & Orchestrator:** Complex longitudinal/lateral control planning, vehicle physics modeling, nuanced cross-branch rebases, final verification before merging into `sg-stinger-combined`. | Mechanical tasks (downloads, log grepping, simple test reruns) that waste high-tier quota. |
| **Claude Sonnet 5.5**<br>`claude-sonnet-5-5` | Claude / Antigravity | Standard context<br>Fast latency | **Precision Implementation:** Writing feature code on `sg-*` branches, refactoring car controller hooks, implementing vehicle state machines, creating unit tests. | Massive uncompressed log digestion where token limits or costs bite. |
| **Claude Haiku 5.5**<br>`claude-haiku-5-5` | Claude | Standard context<br>Ultra-fast | **Mechanical Claude Tasks:** Running scripted replays with fixed arguments, simple doc updates, git branch hygiene checks when working within Claude threads. | Nuanced control logic, complex debugging, or large code refactors. |
| **Gemini 3.8 Flash**<br>`gemini-3.8-flash` | Antigravity | **1M+ context**<br>Ultra-fast latency | **Massive Log Ingestion & Parallel Grunt Work:** Parsing large CAN/rlog dumps, multi-file compiler traces, wide codebase grep sweeps, quick scripts. Runs on Google's quota pool, sparing Claude rate limits. | Extremely subtle multi-file architectural refactoring where Sonnet's code styling is preferred. |
| **Gemini 4 Argon**<br>`gemini-4-argon` | Antigravity | High reasoning compute | **Adversarial Code Review & Math/Physics Audit:** Independent review of tuning parameters against factory ACC data, verifying MPC/acados trajectory math. Catches Claude family blind spots. | High-speed iterative tasks or simple script runs. **Only on the user's request:** the user starts Argon reviews manually, so never delegate to it unasked. |

---

## T3 Code Delegation Patterns

Orchestrators delegate across providers with `delegate_task`. Every delegate gets a proper brief, as `AGENTS.md` requires:

- context already established (branch, base, what was decided)
- exact steps
- constraints
- a time budget
- what to report back

Every brief also states these constraints:

- Pass `STINGER_DEV_HOME=/home/openpilotstuff/starpilot-dev STARPILOT_DIR=/home/openpilotstuff/starpilot` explicitly. Delegate shells may not source `~/.bashrc`.
- Never read, print or copy `/home/openpilotstuff/.comma_jwt`. Downloads go through `comma_dl.py` as described in `AGENTS.md`.
- Never touch `opendbc_repo/opendbc/safety` or `panda/`.
- Never commit or merge into `sg-stinger-combined` without the user's explicit OK.
- Do not create worktrees or branches. Work in the worktree named in the brief.

### Pattern 1: Opus plans, Gemini Flash researches and sweeps (quota saver)
* **Parent:** `claude-opus-5-5` (Claude Code).
* **Delegate:** `gemini-3.8-flash` on `antigravity`.
* **Brief must include:** exact log paths (under `starpilot-dev/logs/`, route names from `starpilot_stinger_dev/routes/ROUTES.md`), the analysis script or query to run (`./run_py.sh analysis/x.py` with args), the numbers to report back, and a time budget.
* **Why:** Ingests huge log outputs without burning Anthropic 5-hour/weekly token quotas.

### Pattern 2: Parallel test and replay runs
* **Delegate:** `gemini-3.8-flash` (or Haiku in Claude threads).
* **Brief must include:**
  - the worktree path (`wt/<branch>/`)
  - the exact command, e.g. `./run_tests.sh <ref> <paths>` from `starpilot_stinger_dev/`
  - serial execution (`-n 0`)
  - the env vars above
  - a time budget
  - a report of pass/fail counts plus the full output of any failure
* **No `isolation: "worktree"`.** `run_tests.sh` takes a ref, and delegate-made checkouts skip the `wt/<name>/` layout and the `AGENTS.md` symlink.
* **Replays:** run one delegate per route in parallel.
* **Why:** Offloads long mechanical runs. The main session reviews the results.

### Review
Independent review (Argon or any other model) is started manually by the user. If asked to prepare one, fix the acceptance criteria and thresholds from measured data (factory ACC logs, replays) in the brief before the review runs, and don't move them afterwards.
