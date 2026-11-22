# Sokoban Solver — Room-by-Room Push-Optimal Search

Entry for make.fun's [Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty ($100 in USDC).

## What it does

`main.ts` (Node 22, no dependencies) reads the five levels of a set from
`/input/<id>/level.txt` and writes `/output/<id>/solution.txt` for each: the
player's moves in `u/d/l/r` that push every box onto a goal in as few pushes
as possible.

## Approach

A level is a one-square hallway with 1–4 independent rooms off it. A box
pushed out of a room's door into the hallway can never be pushed back, and
every goal sits inside the box's own room — so in any optimal solution each
room is solved on its own, and the level's push-optimal count is the sum of
the rooms' optima. The solver therefore:

1. **Splits** the level into rooms (connected floor components once the
   hallway row — the row the player starts on — is removed).
2. **Solves each room push-optimally** with A\* over pushes (state = sorted
   box cells + player cell), using a minimum-cost box-to-goal matching
   heuristic, dead-square pruning from a reverse-pull analysis seeded at the
   goals, and 2×2 freeze-deadlock pruning. A greedy best-first search with
   the same heuristic stands by if A\* ever exhausts its node budget.
3. **Stitches** the rooms together with hallway walks between doors, in
   left-to-right door order.

Search budgets are node counts only, never the wall clock, so two runs
produce byte-identical output. See [STRATEGY.md](STRATEGY.md) for the measured
baseline, the design numbers, and the stress results.

## Results (practice set, kit scorer)

| instance | pushes | P\* | points |
|----------|--------|-----|--------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

**Score 1,000,000** (house baseline plain-astar 1.0.0: 200,000 — it solves
only i001). Whole-set runtime ≈ 0.6 s of the 60 s wall budget. Six further
freshly drawn sets (new seeds, unseen levels) all score 1,000,000 too.

## Run it locally

```sh
# from the unpacked kit directory
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node /path/to/main.ts
# score the outputs with the kit scorer
(cd program/scorer && MAKEFUN_OUTPUT_DIR=out MAKEFUN_REFERENCE_DIR=practice/reference MAKEFUN_SCORES_DIR=scores node score.ts)
```

## Files

- `main.ts` — the solver (the whole entry).
- `STRATEGY.md` — baseline, design, and measured results.
- `LICENSE` — MIT.
