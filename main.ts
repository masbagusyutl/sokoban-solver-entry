// Room-by-room push-optimal Sokoban solver.
//
// Idea (from the level construction, not from any other solver): a level is a
// one-square hallway with 1-4 independent rooms off it. A box pushed out of a
// room's door into the hallway can never be pushed back, and every goal sits
// inside the box's own room, so in any optimal solution each room is solved
// on its own and the level's push-optimal count is the sum of the rooms'.
// This program therefore splits the level into rooms (connected floor
// components once the hallway row is removed), solves each room push-optimally
// with A* over pushes, and stitches the rooms together with hallway walks.
//
// Per-room search: A* over (sorted box cells, player cell). Successors are
// pushes; player reachability between pushes is a flood fill. The heuristic
// is the minimum-cost matching of boxes to goals under shortest-path distance
// (each push moves one box one square, so the matching sum lower-bounds the
// pushes left). Dead states are pruned with (a) dead squares from a reverse
// pull analysis seeded at the goals and (b) 2x2 blocks of wall/box cells
// holding a box and no goal, in which no box can ever move. Search budgets
// are node counts only, never the clock, so two runs produce byte-identical
// output.
//
// Node 22, TypeScript with erasable syntax only, no dependencies.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions: 0 = up, 1 = down, 2 = left, 3 = right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];

// Deterministic, count-based search budgets (never the wall clock).
const ASTAR_CAP = 250000;
const GREEDY_CAP = 250000;
const GLOBAL_CAP = 1500000;

let expansions = 0;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goal: Uint8Array;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// Breadth-first distances from `from` over cells where `open[i]` is 1.
// Returns an Int32Array of distances (-1 = unreachable).
function bfsDist(w: number, open: Uint8Array, from: number): Int32Array {
  const dist = new Int32Array(open.length).fill(-1);
  const q = new Int32Array(open.length);
  let head = 0;
  let tail = 0;
  dist[from] = 0;
  q[tail++] = from;
  while (head < tail) {
    const c = q[head++];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= open.length || !open[n] || dist[n] !== -1) continue;
      dist[n] = dist[c] + 1;
      q[tail++] = n;
    }
  }
  return dist;
}

// Shortest walk (as a move string) from `from` to `to` over `open` cells,
// avoiding `blocked` cells. Fixed neighbour order, so the result is unique.
function bfsPath(
  w: number,
  open: Uint8Array,
  blocked: Uint8Array,
  from: number,
  to: number,
): string | null {
  if (from === to) return '';
  const prev = new Int32Array(open.length).fill(-2);
  const prevDir = new Int8Array(open.length);
  const q = new Int32Array(open.length);
  let head = 0;
  let tail = 0;
  prev[from] = -1;
  q[tail++] = from;
  while (head < tail) {
    const c = q[head++];
    if (c === to) break;
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= open.length || !open[n] || blocked[n] || prev[n] !== -2) continue;
      prev[n] = c;
      prevDir[n] = d;
      q[tail++] = n;
    }
  }
  if (prev[to] === -2) return null;
  let out = '';
  let c = to;
  while (c !== from) {
    out = CH[prevDir[c]] + out;
    c = prev[c];
  }
  return out;
}

// Reachable set from `from` over `open` cells avoiding `blocked` cells.
function reachableSet(
  w: number,
  open: Uint8Array,
  blocked: Uint8Array,
  from: number,
): Uint8Array {
  const seen = new Uint8Array(open.length);
  const q = new Int32Array(open.length);
  let head = 0;
  let tail = 0;
  seen[from] = 1;
  q[tail++] = from;
  while (head < tail) {
    const c = q[head++];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0) continue;
      const n = ny * w + nx;
      if (n >= open.length || !open[n] || blocked[n] || seen[n]) continue;
      seen[n] = 1;
      q[tail++] = n;
    }
  }
  return seen;
}

interface Problem {
  w: number;
  // Cells a box may occupy (the room's floor).
  boxOpen: Uint8Array;
  // Cells the player may walk on (room floor plus, for a room, its door's
  // hallway square).
  walkOpen: Uint8Array;
  goals: number[];
  goalSet: Uint8Array;
  boxes0: number[];
  player0: number;
  // The square the player must be able to reach once the boxes are home
  // (-1 when there is none, i.e. the whole-level fallback).
  exit: number;
  // dist[g][cell]: shortest box-travel distance from cell to goal g.
  dist: Int32Array[];
  // live[i] = 1 when a box on cell i can still reach some goal (reverse-pull
  // analysis seeded at the goals).
  live: Uint8Array;
}

function buildProblem(
  level: Level,
  region: number[],
  walkExtra: number[],
  boxes0: number[],
  player0: number,
  exit: number,
): Problem {
  const { w, h, goal } = level;
  const size = w * h;
  const boxOpen = new Uint8Array(size);
  const walkOpen = new Uint8Array(size);
  for (const c of region) {
    boxOpen[c] = 1;
    walkOpen[c] = 1;
  }
  for (const c of walkExtra) walkOpen[c] = 1;
  const goals = region.filter((c) => goal[c] === 1);
  // Shortest box-travel distances: BFS from each goal over the region.
  const dist = goals.map((g) => bfsDist(w, boxOpen, g));
  // Dead squares: reverse the pushes. A box on goal g is live; if a box on
  // cell c is live, a box on a neighbour n of c is live too when the player
  // could have stood on the far side of n to push it to c, i.e. when the
  // cell past n away from c is walkable (a relaxation: reachability of that
  // stance is ignored, so `live` is a superset of the truly live cells and
  // pruning a box off a non-live cell is sound).
  const live = new Uint8Array(size);
  const stack: number[] = [];
  for (const g of goals) {
    live[g] = 1;
    stack.push(g);
  }
  while (stack.length > 0) {
    const c = stack.pop() as number;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!boxOpen[n] || live[n]) continue;
      // The cell past n, away from c, must be walkable for the push n -> c.
      const bx = nx + DX[d];
      const by = ny + DY[d];
      if (bx < 0 || by < 0 || bx >= w || by >= h) continue;
      const b = by * w + bx;
      if (!walkOpen[b]) continue;
      live[n] = 1;
      stack.push(n);
    }
  }
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  return { w, boxOpen, walkOpen, goals, goalSet, boxes0, player0, exit, dist, live };
}

function keyOf(boxes: number[], player: number): string {
  return boxes.join(',') + '|' + player;
}

// Minimum-cost matching of boxes to goals; Infinity when no box-to-goal
// bijection is reachable even ignoring the other boxes.
function heuristic(prob: Problem, boxes: number[]): number {
  const n = boxes.length;
  const goals = prob.goals;
  if (n === 0) return 0;
  // Brute-force permutations (n <= 4 in every room this program is built for;
  // the fallback caps at the same bound by construction of the levels).
  const perm = goals.map((_, i) => i);
  let best = Infinity;
  const rec = (k: number, sum: number): void => {
    if (sum >= best) return;
    if (k === n) {
      best = sum;
      return;
    }
    for (let i = k; i < n; i++) {
      const t = perm[k];
      perm[k] = perm[i];
      perm[i] = t;
      const d = prob.dist[perm[k]][boxes[k]];
      if (d !== -1) rec(k + 1, sum + d);
      const t2 = perm[k];
      perm[k] = perm[i];
      perm[i] = t2;
    }
  };
  rec(0, 0);
  return best;
}

// True when some 2x2 block with no goal cell is all wall-or-box and holds a
// box: no box in such a block can ever move (every push direction either
// targets a wall/box inside the block or needs a stance inside it).
function hasFrozenBlock(
  prob: Problem,
  boxes: number[],
  boxSet: Uint8Array,
): boolean {
  const { w, goalSet } = prob;
  const size = prob.boxOpen.length;
  const rows = size / w;
  // A cell counts as blocked when it is not walkable floor, or a box sits
  // on it; walkable floor without a box is open.
  const cellBlocked = (c: number): boolean =>
    prob.walkOpen[c] === 0 || boxSet[c] === 1;
  for (let y = 0; y + 1 < rows; y++) {
    for (let x = 0; x + 1 < w; x++) {
      const i = y * w + x;
      const j = i + 1;
      const k = i + w;
      const l = i + w + 1;
      if (goalSet[i] || goalSet[j] || goalSet[k] || goalSet[l]) continue;
      if (
        cellBlocked(i) &&
        cellBlocked(j) &&
        cellBlocked(k) &&
        cellBlocked(l) &&
        (boxSet[i] || boxSet[j] || boxSet[k] || boxSet[l])
      ) {
        return true;
      }
    }
  }
  return false;
}

interface Push {
  boxFrom: number;
  dir: number;
}

interface SearchResult {
  pushes: Push[];
  expanded: number;
}

// A* (or greedy best-first when `greedy`) over pushes. Returns the
// push-optimal push list, or null when the budget runs out.
function search(prob: Problem, cap: number, greedy: boolean): SearchResult | null {
  const n = prob.boxes0.length;
  const startBoxes = [...prob.boxes0].sort((a, b) => a - b);
  const startKey = keyOf(startBoxes, prob.player0);
  const h0 = heuristic(prob, startBoxes);
  if (h0 === Infinity) return null;

  // Binary heap ordered by (f, h, seq) — fully deterministic.
  interface HeapEntry {
    f: number;
    h: number;
    seq: number;
    key: string;
    g: number;
  }
  const heap: HeapEntry[] = [];
  let seq = 0;
  const pushHeap = (e: HeapEntry): void => {
    heap.push(e);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      const a = heap[i];
      const b = heap[p];
      if (
        a.f < b.f ||
        (a.f === b.f && (a.h < b.h || (a.h === b.h && a.seq < b.seq)))
      ) {
        heap[i] = b;
        heap[p] = a;
        i = p;
      } else break;
    }
  };
  const popHeap = (): HeapEntry => {
    const top = heap[0];
    const last = heap.pop() as HeapEntry;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        const better = (x: HeapEntry, y: HeapEntry): boolean =>
          x.f < y.f || (x.f === y.f && (x.h < y.h || (x.h === y.h && x.seq < y.seq)));
        if (l < heap.length && better(heap[l], heap[m])) m = l;
        if (r < heap.length && better(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i];
        heap[i] = heap[m];
        heap[m] = t;
        i = m;
      }
    }
    return top;
  };

  const bestG = new Map<string, number>();
  const parent = new Map<string, { prev: string; push: Push }>();
  const stateBoxes = new Map<string, number[]>();
  const statePlayer = new Map<string, number>();
  bestG.set(startKey, 0);
  stateBoxes.set(startKey, startBoxes);
  statePlayer.set(startKey, prob.player0);
  pushHeap({ f: greedy ? h0 : h0, h: h0, seq: seq++, key: startKey, g: 0 });

  let expanded = 0;
  const boxSet = new Uint8Array(prob.boxOpen.length);
  const blocked = new Uint8Array(prob.boxOpen.length);

  while (heap.length > 0) {
    if (expanded >= cap || expansions >= GLOBAL_CAP) return null;
    const cur = popHeap();
    const g = bestG.get(cur.key);
    if (g === undefined || g !== cur.g) continue;
    const boxes = stateBoxes.get(cur.key) as number[];
    const player = statePlayer.get(cur.key) as number;
    expanded++;
    expansions++;

    // Goal: every box on a goal and the player able to reach the exit.
    let solved = true;
    for (const b of boxes) {
      if (!prob.goals.includes(b)) {
        solved = false;
        break;
      }
    }
    if (solved) {
      if (prob.exit === -1) {
        return { pushes: reconstruct(parent, cur.key), expanded };
      }
      boxSet.fill(0);
      for (const b of boxes) boxSet[b] = 1;
      blocked.set(boxSet);
      const reach = reachableSet(prob.w, prob.walkOpen, blocked, player);
      if (reach[prob.exit]) {
        return { pushes: reconstruct(parent, cur.key), expanded };
      }
      // Boxes home but the player is shut in: keep searching.
    }

    // Player flood fill from the current square, boxes as obstacles.
    boxSet.fill(0);
    for (const b of boxes) boxSet[b] = 1;
    blocked.set(boxSet);
    const reach = reachableSet(prob.w, prob.walkOpen, blocked, player);

    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi];
      const bx = b % prob.w;
      const by = (b / prob.w) | 0;
      for (let d = 0; d < 4; d++) {
        const tx = bx + DX[d];
        const ty = by + DY[d];
        const sx = bx - DX[d];
        const sy = by - DY[d];
        if (tx < 0 || ty < 0 || sx < 0 || sy < 0) continue;
        const target = ty * prob.w + tx;
        const stance = sy * prob.w + sx;
        if (target >= prob.boxOpen.length || stance >= prob.boxOpen.length) continue;
        if (!prob.boxOpen[target] || boxSet[target]) continue;
        if (!prob.walkOpen[stance] || !reach[stance]) continue;
        const next = [...boxes];
        next[bi] = target;
        next.sort((a, b2) => a - b2);
        // Dead-square prune: a box off every goal's pull region is lost.
        let dead = false;
        for (const nb of next) {
          if (!prob.live[nb]) {
            dead = true;
            break;
          }
        }
        if (dead) continue;
        const nKey = keyOf(next, b);
        const ng = cur.g + 1;
        const prevG = bestG.get(nKey);
        if (prevG !== undefined && prevG <= ng) continue;
        const nh = heuristic(prob, next);
        if (nh === Infinity) continue;
        const nBoxSet = new Uint8Array(prob.boxOpen.length);
        for (const nb of next) nBoxSet[nb] = 1;
        if (hasFrozenBlock(prob, next, nBoxSet)) continue;
        bestG.set(nKey, ng);
        stateBoxes.set(nKey, next);
        statePlayer.set(nKey, b);
        parent.set(nKey, { prev: cur.key, push: { boxFrom: b, dir: d } });
        pushHeap({
          f: greedy ? nh : ng + nh,
          h: nh,
          seq: seq++,
          key: nKey,
          g: ng,
        });
      }
    }
  }
  return null;
}

function reconstruct(
  parent: Map<string, { prev: string; push: Push }>,
  key: string,
): Push[] {
  const out: Push[] = [];
  let k = key;
  while (parent.has(k)) {
    const step = parent.get(k) as { prev: string; push: Push };
    out.push(step.push);
    k = step.prev;
  }
  out.reverse();
  return out;
}

// Expand a room's push list into step moves. The player starts on `entry`
// (the door's hallway square) and the walk ends back there.
function expandRoomMoves(
  prob: Problem,
  pushes: Push[],
): { moves: string; pushes: number } | null {
  const boxes = [...prob.boxes0].sort((a, b) => a - b);
  let player = prob.player0;
  let moves = '';
  const blocked = new Uint8Array(prob.boxOpen.length);
  for (const p of pushes) {
    const stance = p.boxFrom - DY[p.dir] * prob.w - DX[p.dir];
    blocked.fill(0);
    for (const b of boxes) blocked[b] = 1;
    const walk = bfsPath(prob.w, prob.walkOpen, blocked, player, stance);
    if (walk === null) return null;
    moves += walk;
    moves += CH[p.dir];
    const bi = boxes.indexOf(p.boxFrom);
    if (bi === -1) return null;
    boxes[bi] = p.boxFrom + DY[p.dir] * prob.w + DX[p.dir];
    boxes.sort((a, b) => a - b);
    player = p.boxFrom;
  }
  // Walk back out to the door's hallway square.
  blocked.fill(0);
  for (const b of boxes) blocked[b] = 1;
  const out = bfsPath(prob.w, prob.walkOpen, blocked, player, prob.exit);
  if (out === null) return null;
  moves += out;
  return { moves, pushes: pushes.length };
}

interface Room {
  region: number[];
  boxes: number[];
  door: number;
  doorHall: number;
}

// Split the level into rooms: connected floor components once the hallway
// row (the row the player starts on) is removed. Returns null when the
// level does not fit that shape, in which case the caller falls back to a
// whole-level search.
function decompose(level: Level): Room[] | null {
  const { w, h, wall, boxes, player } = level;
  const hallRow = (player / w) | 0;
  const inHall = (c: number): boolean => ((c / w) | 0) === hallRow;
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  let boxesSeen = 0;
  for (let s = 0; s < w * h; s++) {
    if (wall[s] || inHall(s) || seen[s]) continue;
    // Flood-fill one component.
    const region: number[] = [];
    const stack = [s];
    seen[s] = 1;
    while (stack.length > 0) {
      const c = stack.pop() as number;
      region.push(c);
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || inHall(n) || seen[n]) continue;
        seen[n] = 1;
        stack.push(n);
      }
    }
    const rBoxes = boxes.filter((b) => region.includes(b));
    const rGoals = region.filter((c) => level.goal[c] === 1);
    if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) return null;
    boxesSeen += rBoxes.length;
    // The door: the room cell touching a hallway floor cell.
    const doors = region.filter((c) => {
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (!wall[n] && inHall(n)) return true;
      }
      return false;
    });
    if (doors.length !== 1) return null;
    const door = doors[0];
    const dx = door % w;
    const dy = (door / w) | 0;
    let doorHall = -1;
    for (let d = 0; d < 4; d++) {
      const nx = dx + DX[d];
      const ny = dy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!wall[n] && inHall(n)) {
        doorHall = n;
        break;
      }
    }
    if (doorHall === -1) return null;
    rooms.push({ region, boxes: rBoxes, door, doorHall });
  }
  if (boxesSeen !== boxes.length) return null;
  // Left-to-right door order: the order the rooms are visited in.
  rooms.sort((a, b) => a.doorHall - b.doorHall);
  return rooms;
}

function solveLevel(level: Level): { moves: string; pushes: number; perRoom: number[] } | null {
  const rooms = decompose(level);
  if (rooms !== null) {
    let moves = '';
    let player = level.player;
    let totalPushes = 0;
    const perRoom: number[] = [];
    const allBoxes = [...level.boxes];
    const blocked = new Uint8Array(level.w * level.h);
    for (const room of rooms) {
      // Walk the hallway from the current square to this room's door.
      blocked.fill(0);
      for (const b of allBoxes) blocked[b] = 1;
      const walkOpen = new Uint8Array(level.w * level.h);
      for (let i = 0; i < walkOpen.length; i++) {
        if (!level.wall[i]) walkOpen[i] = 1;
      }
      const toDoor = bfsPath(level.w, walkOpen, blocked, player, room.doorHall);
      if (toDoor === null) return null;
      moves += toDoor;
      player = room.doorHall;

      const prob = buildProblem(level, room.region, [room.doorHall], room.boxes, room.doorHall, room.doorHall);
      let res = search(prob, ASTAR_CAP, false);
      if (res === null) res = search(prob, GREEDY_CAP, true);
      if (res === null) {
        console.error(`room at door ${room.door} unsolved`);
        return null;
      }
      const expanded = expandRoomMoves(prob, res.pushes);
      if (expanded === null) return null;
      moves += expanded.moves;
      totalPushes += expanded.pushes;
      perRoom.push(expanded.pushes);
      player = room.doorHall;
      // The room's boxes are now on its goals: replay this room's pushes to
      // get their final cells and refresh the shared box list.
      const rb = [...room.boxes].sort((a, b) => a - b);
      for (const p of res.pushes) {
        const bi = rb.indexOf(p.boxFrom);
        rb[bi] = p.boxFrom + DY[p.dir] * level.w + DX[p.dir];
        rb.sort((a, b) => a - b);
      }
      for (let i = 0; i < allBoxes.length; i++) {
        if (room.boxes.includes(allBoxes[i])) {
          allBoxes[i] = rb.shift() as number;
        }
      }
    }
    return { moves, pushes: totalPushes, perRoom };
  }

  // Fallback: one A* over the whole level (no exit requirement).
  const region: number[] = [];
  for (let i = 0; i < level.w * level.h; i++) {
    if (!level.wall[i]) region.push(i);
  }
  const prob = buildProblem(level, region, [], level.boxes, level.player, -1);
  let res = search(prob, ASTAR_CAP, false);
  if (res === null) res = search(prob, GREEDY_CAP, true);
  if (res === null) return null;
  const expanded = expandRoomMoves(prob, res.pushes);
  if (expanded === null) return null;
  return { moves: expanded.moves, pushes: expanded.pushes, perRoom: [expanded.pushes] };
}

for (const id of readdirSync(INPUT_DIR).sort()) {
  const level = parseLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'));
  const solved = solveLevel(level);
  const moves = solved === null ? '' : solved.moves;
  if (solved !== null) {
    console.error(`${id}: solved in ${solved.pushes} pushes (rooms: ${solved.perRoom.join('+')}), ${expansions} expansions so far`);
  } else {
    console.error(`${id}: UNSOLVED`);
  }
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
