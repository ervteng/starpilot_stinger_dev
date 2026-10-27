# Kia Stinger 2022 StarPilot workspace: rules for every agent

This is ervteng's Kia Stinger 2022 fork of StarPilot (FireStar's openpilot fork). Ubuntu 26.04 on WSL2, root.
These rules apply to every coding agent: Claude Code, Gemini/Antigravity, Codex and others.
Current state and open work: `/home/openpilotstuff/HANDOVER.md`. Read it before starting.

## Rules
- **The daily build is `stinger-combined`.** Nothing goes into it without the user's explicit OK, and that includes your own fixes. Work on a separate branch and ask first.
- **Commit identity.** Author is the user, committer is you:
  `GIT_AUTHOR_NAME=ervteng GIT_AUTHOR_EMAIL=5085265+ervteng@users.noreply.github.com GIT_COMMITTER_NAME=<you> GIT_COMMITTER_EMAIL=<you> git commit ...`
  End each message with a `Co-Authored-By:` trailer for your model.
- **Push** with `git push -u origin <branch>`. `gh` is logged in as ervteng.
- **Never force-push, with one exception.** Never rewrite pushed history; use revert commits instead. The exception is SG base updates: when the user asks to move to a newer SG tip, rebase the `sg-*` feature branches onto it and rebuild `sg-stinger-combined` (fresh `--no-ff` merges on the new tip). Then push those branches with `--force-with-lease`. Check with `git range-diff` that every commit is unchanged or that conflict fixes are reported. The user does this often.
- **Never use bare `git stash`.** Worktrees share the stash. Use WIP commits.
- **No PRs to FireStar** (`firestar5683/StarPilot`) unless the user asks.
- **Never delete remote branches or tags without the user's OK.** To retire a branch, tag it `archive/<name>` first.
- **The comma token** is in `/home/openpilotstuff/.comma_jwt`. Never read, print or copy it. Only source it inside download scripts:
  `bash -c "set -a; . /home/openpilotstuff/.comma_jwt; set +a; ./comma_dl.py ..."`
- **Panda safety.** Don't touch `opendbc_repo/opendbc/safety` or `panda/` (comma ban risk).

## How to work
- Gate Stinger-only behaviour to `KIA_STINGER_2022`, not to all cars.
- Every tuning number must come from measured data (factory ACC logs, replays), with its basis in a comment. Fix acceptance criteria before running an eval, and don't move them afterwards.
- Reuse existing StarPilot/SG per-car hooks before writing new mechanisms. One mechanism per commit.
- **SecretGoodStarPilot (SG) base:** branch new SG work from the base `sg-stinger-combined` is built on (currently `0422970f2d`). Don't move to a newer SG tip unless the user asks.
- **Each SG change gets its own branch off that base,** suitable for an upstream PR. Then merge it with `--no-ff` into `sg-stinger-combined`.
- **Size validation to the change.** For small edits, run only the affected test files. Run the full serial SG suite once per feature branch, before pushing or merging. Run replays per route in parallel.

## Layout (`/home/openpilotstuff`)
| Path | What |
|---|---|
| `starpilot/` | clone of github.com/ervteng/StarPilot (branch `StarPilot`) |
| `wt/<branch>/` | git worktrees of StarPilot, one per active branch |
| `starpilot_stinger_dev/` | clone of github.com/ervteng/starpilot_stinger_dev (public): analysis, downloaders, test runner, route catalog, docs |
| `starpilot-dev/` | `$STINGER_DEV_HOME`: `venv/` + `x86/` (old toolchain, keep at `f6f89ce96`), `venv-sg/` + `x86-sg/` (SG toolchain), `logs/`, `video/` |

- **Env vars:** pass `STINGER_DEV_HOME=/home/openpilotstuff/starpilot-dev STARPILOT_DIR=/home/openpilotstuff/starpilot` explicitly. Agent shells may not source `~/.bashrc`, and the scripts would then fall back to `~/stinger`.
- **Toolchain:** `STINGER_TOOLCHAIN=old|sg`. Run Python tools with `./run_py.sh analysis/x.py`, and tests with `./run_tests.sh <ref> <paths>`. SG tests need `-n 0` (serial). See the dev repo README, "Toolchains".
- **Downloads:** `comma_dl.py ROUTE SEGS --type rlog|qlog|qcamera|fcamera`. Dongle `59fa1d41318b87d3`. Catalog: `starpilot_stinger_dev/routes/ROUTES.md`.
- **New worktree:** after `git worktree add`, link the rules into its root so Antigravity finds them: `ln -s ../../AGENTS.md wt/<name>/AGENTS.md`. Git already ignores it through `.git/info/exclude`.
- **glibc gotcha:** a rebuilt `x86*/third_party/acados/x86_64/lib/libblasfeo.so` needs `patchelf --clear-execstack`.

## Keep docs current
At the end of a task, update `HANDOVER.md`: current state only, and keep it short. Move finished detail into `starpilot_stinger_dev/docs/history.md`. Keep this file under about 8 KB; some tools cap rules files at 24 KB. A copy is committed as `starpilot_stinger_dev/docs/AGENTS.md` so a new machine can recreate it. When you change this file, update the copy too.
