# STRATEGY.md — Sokoban solver: room-by-room push-optimal search

## 1. The scoring, and what it demands

Each instance is worth 1,000 points: full marks only at exactly P* pushes
(the level's push-optimal count), `500 * P* / pushes` for anything more, 0
for an unsolved level. Score is the mean over 5 instances, ×1,000. A reward
needs a score **strictly above the house baseline**, make.fun's plain-astar
1.0.0. Every instance must be answered, and two executions must agree byte
for byte (determinism), inside one 60 s wall budget for the whole set.

## 2. Measured house baseline (kit scorer, practice set)

Ran the kit's `baselines/plain-astar/main.ts` on the practice set, then the
kit scorer (`program/scorer/score.ts`):

| instance | P* | baseline pushes | baseline points |
|----------|----|-----------------|-----------------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | — (empty) | 0 |
| i003 | 29 | — (empty) | 0 |
| i004 | 47 | — (empty) | 0 |
| i005 | 60 | — (empty) | 0 |

**Baseline score: 200,000.** It solves the 3-box room and gives up on every
4-box room: plain A* over the whole level, with no deadlock analysis, cannot
cope once a second room multiplies the state space. That is the gap to beat,
and it is wide: solving i002 as well already doubles the score.

## 3. Design: decompose, then push-optimal A* per room

The generator builds a level as a hallway plus rooms, each room solved
push-optimally on its own, P* being the sum of the room optima. Two facts
follow from the construction and are load-bearing for this entry:

- A box pushed through a door into the hallway can never be pushed back, so
  an optimal solution never moves a box out of its room.
- Goals are per-room, so rooms are independent problems joined only by the
  player's walk between doors.

So the solver splits the level into rooms (connected floor components once
the hallway row — the row the player starts on — is removed; each component
must hold equal boxes and goals and exactly one door cell) and solves each
room independently. If a level ever fails that shape check, a whole-level A*
with the same machinery is the fallback.

Per-room search (all written fresh for this entry):

- **State**: sorted box cells + player cell. **Moves**: pushes only; player
  reachability between pushes is a flood fill.
- **Heuristic**: minimum-cost matching of boxes to goals under shortest-path
  (box-travel) distance. One push moves one box one square, so the matching
  sum lower-bounds the pushes remaining — admissible, and consistent.
- **Deadlock pruning**: (a) dead squares from a reverse-pull analysis seeded
  at the goals — a box on a cell no goal can pull from is lost; (b) 2×2
  blocks of wall/box cells holding a box and no goal, in which no box can
  ever move (every push direction targets a wall/box inside the block or
  needs a stance inside it). Both are sound: neither prunes a solvable state.
- **Goal test**: all boxes on goals *and* the player able to walk back out
  to the door (the generator guarantees goals never wall the player in, but
  the test is enforced anyway).
- **Second strategy (pattern: test more than one approach)**: a greedy
  best-first search on the same heuristic stands by if A* exhausts its node
  budget. On every set tested, A* finished first and the fallback never
  fired — the submitted behaviour is A* throughout.
- **Budgets are node counts only** (250k expansions per room per strategy,
  1.5M total), never the clock, so output is deterministic by construction.
  No clock reads, no randomness, fixed neighbour order everywhere.

Stitching: rooms are visited in left-to-right door order; the player walks
the hallway between doors (BFS avoiding boxes), enters, executes the room's
push list expanded to steps (BFS walk to each stance + the push step), and
walks back out to the door.

## 4. Measured results

Practice set (kit scorer), runtime 0.61 s, 7,279 expansions total:

| instance | rooms (pushes) | total | P* | points |
|----------|---------------|-------|----|--------|
| i001 | 13 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 16 | 1,000 |
| i003 | 14+15 | 29 | 29 | 1,000 |
| i004 | 17+15+15 | 47 | 47 | 1,000 |
| i005 | 14+16+16+14 | 60 | 60 | 1,000 |

**Practice score: 1,000,000 vs baseline 200,000 — a 5× margin, at P* on
every instance.** Each room's push count matches its room optimum (the sums
equal the manifest P*, which is the sum of the room optima), so every room
is provably push-optimal, not merely solved.

## 5. Stress tests (held-out sets, same generator, new seeds)

The scored set is drawn after entries close from a round nobody can know
yet, so the practice set alone is not enough evidence. Six fresh sets were
drawn with the kit generator (`program/generator/draw.ts`, new seeds) and
solved + scored with the kit scorer:

| set | i001 | i002 | i003 | i004 | i005 | score | wall |
|-----|------|------|------|------|------|-------|------|
| a1 | 15/15 | 15/15 | 28/28 | 43/43 | 62/62 | 1,000,000 | 1.15 s |
| b2 | 12/12 | 14/14 | 32/32 | 45/45 | 60/60 | 1,000,000 | 1.10 s |
| c3 | 12/12 | 16/16 | 31/31 | 44/44 | 64/64 | 1,000,000 | 1.08 s |
| d4 | 12/12 | 19/19 | 30/30 | 50/50 | 63/63 | 1,000,000 | 0.47 s |
| e5 | 14/14 | 14/14 | 36/36 | 43/43 | 61/61 | 1,000,000 | 0.47 s |
| f6 | 13/13 | 14/14 | 28/28 | 45/45 | 64/64 | 1,000,000 | 1.54 s |

(pushes shown as solved/P*; all 30 instances at P*.) Worst set used 17,465
expansions; worst wall time 1.54 s of the 60 s budget — a ~40× time margin.
The hardest single room seen needed 22 pushes (set e5, i003) and still solved
inside the A* budget with room to spare.

## 6. Determinism

Two runs on the practice set produce byte-identical `solution.txt` files
(verified with `diff -r`). No time-based cutoffs, no randomness.

## 7. What could still go wrong, and the mitigations

- **A room harder than any seen** (e.g. a 20+ push room on the scored set):
  the per-room A* cap is 250k expansions; the worst room observed used an
  order of magnitude less. If A* ever caps out, the greedy fallback runs,
  and a merely-solved (non-optimal) room still scores `500*P*/pushes`
  instead of 0.
- **A level that does not decompose** (shape check fails): the whole-level
  A* fallback runs; it solves single-room levels of practice size, though
  it would likely time out on multi-room levels — this shape has never
  occurred on any generated set (the generator guarantees it).
- **Room mis-detection**: the shape check (equal boxes/goals per component,
  exactly one door, all boxes accounted for) refuses to guess; a failed
  check falls back rather than emitting a wrong decomposition.
