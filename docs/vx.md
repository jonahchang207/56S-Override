---
layout: default
title: Branch Manager
---

# Branch Manager (`vx`)

`vx` is a local CLI tool that enforces the two-branch workflow used by Team 56S-Override. It is the only sanctioned path for promoting code from development into production.

---

## Branch strategy

| Branch | Purpose | Who writes here |
|---|---|---|
| `main` | Production — what runs at competition | `vx` deploy only |
| `dev` | Active development — all day-to-day work | Everyone |

`main` is never committed to directly. The script guards all three destructive operations (commit, deploy, rollback) against running on the wrong branch.

---

## Running the tool

From the repo root:

```bash
./vx
```

An interactive menu appears:

```
  56S-Override Branch Manager
  ──────────────────────────────
  1) Commit to dev
  2) Deploy → main
  3) Rollback dev
  4) Status
  q) Quit
```

All options return to the menu after completing. Press `q` to exit.

---

## Options

### 1 — Commit to dev

Stages all local changes (`git add -A`), prompts for a commit message, then commits and pushes to `origin/dev`.

```
  Commit message: fix turn PID overshoot on long arcs
  [dev a3f1b2c] fix turn PID overshoot on long arcs
```

**Guard:** aborts if you are not on the `dev` branch.

---

### 2 — Deploy → main

Shows exactly which commits will ship, asks for confirmation, then:

1. Pushes the current `dev` state to `origin/dev`
2. Fast-forwards `main` to the tip of `origin/main`
3. Merges `dev` → `main` with `--no-ff` (preserves branch history)
4. Pushes `main` to `origin/main`
5. Switches you back to `dev`

```
  5 commit(s) ahead of main:
    a3f1b2c fix turn PID overshoot on long arcs
    9d02e4a feat: add path smoothing toggle
    ...

  Deploy to production? [y/N] y
  Deployed successfully. Back on dev.
```

**Guards:**
- Aborts if not on `dev`
- Exits early with no changes if `dev` is already up to date with `main`

---

### 3 — Rollback dev

Shows the last 10 commits on `dev`, asks how many to revert, then creates a `git revert` commit that undoes those changes without rewriting history.

```
  Recent commits on dev:
    a3f1b2c fix turn PID overshoot on long arcs
    9d02e4a feat: add path smoothing toggle
    ...

  How many commits to revert? [1] 2
  Revert last 2 commit(s) on dev? [y/N] y
  Reverted 2 commit(s). History preserved on dev.
```

Rollback uses `git revert --no-commit HEAD~n..HEAD` followed by a single revert commit. It is non-destructive — commits are never deleted from the tree.

**Guard:** aborts if not on `dev`.

---

### 4 — Status

Prints a read-only summary:

```
  Branch: dev

  dev (last 5):
    a3f1b2c fix turn PID overshoot on long arcs
    9d02e4a feat: add path smoothing toggle
    ...

  main (last 5):
    6efafec feat: add vx branch manager CLI script
    3893d90 docs: rewrite README in plain language
    ...

  dev is 5 commit(s) ahead of main
```

---

## Safety model

| Risk | How `vx` handles it |
|---|---|
| Committing directly to `main` | All write operations check `current_branch == dev` |
| Deploying without reviewing changes | Deploy prints pending commits and requires `y` before proceeding |
| Losing work on rollback | Rollback uses `git revert` — commits stay in history |
| Deploying an empty diff | Deploy exits early if `dev` has no new commits over `main` |

---

## Typical workflow

```bash
# --- normal development cycle ---
./vx           # open menu
# 1) Commit to dev  →  commit your changes
# 1) Commit to dev  →  commit more changes

# --- ready to ship ---
# 2) Deploy → main  →  review commits, confirm, done
```
