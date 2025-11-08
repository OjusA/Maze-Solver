"""
Maze Solver: BFS, Dijkstra, and A* with Pygame Visualization

Controls
--------
Left Click   : Toggle wall
Right Click  : Set Start (first) and End (second)
B            : Run Breadth-First Search (BFS)
D            : Run Dijkstra's Algorithm
A            : Run A* Search (Manhattan heuristic)
G            : Generate a random maze
C            : Clear search (keep walls, start/end)
R            : Reset everything (clear walls & search)
S            : Save current grid to grid.json
L            : Load grid.json if present
+ / -        : Increase / decrease animation speed
ESC or Q     : Quit

Outcome
-------
Visual, step-by-step demonstration of each algorithm exploring the maze
and reconstructing the shortest path (if one exists).

Notes
-----
- This is grid-based (4-neighborhood). You can adapt it to 8-neighborhood
  by adjusting neighbors().
- For robotics, you can load an occupancy grid into `grid` and run any solver.
"""
from __future__ import annotations
import json
import math
import os
import random
import sys
from collections import deque
from dataclasses import dataclass
from typing import Dict, Generator, Iterable, List, Optional, Tuple

import pygame

# ----------------------- Config -----------------------
WIDTH, HEIGHT = 900, 700
ROWS, COLS = 35, 45  # Adjust to taste; cells will be ~20px
MARGIN = 1           # gap between cells in pixels
CELL_W = (WIDTH // COLS)
CELL_H = (HEIGHT // ROWS)
SPEED_FPS = 60       # default animation speed

# Ensure cells fit nicely
WIDTH = COLS * CELL_W
HEIGHT = ROWS * CELL_H

# Colors
WHITE = (245, 245, 245)
BLACK = (30, 30, 30)
GRID = (60, 60, 60)
WALL = (35, 35, 35)
START = (255, 165, 0)       # orange
END = (64, 224, 208)        # turquoise
OPEN = (65, 105, 225)       # royal blue (frontier)
CLOSED = (123, 104, 238)    # medium slate blue (visited)
PATH = (50, 205, 50)        # lime green
TEXT = (230, 230, 230)

# Cell states
EMPTY = 0
WALL_S = 1
START_S = 2
END_S = 3
OPEN_S = 4
CLOSED_S = 5
PATH_S = 6

# ----------------------- Utilities -----------------------
Coord = Tuple[int, int]  # (r, c)


def in_bounds(r: int, c: int) -> bool:
    return 0 <= r < ROWS and 0 <= c < COLS


def neighbors_4(r: int, c: int) -> Iterable[Coord]:
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if in_bounds(nr, nc):
            yield (nr, nc)


# Manhattan heuristic for A*
def h_manhattan(a: Coord, b: Coord) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# Reconstruct path from `came_from` map

def reconstruct_path(came_from: Dict[Coord, Coord], start: Coord, goal: Coord) -> List[Coord]:
    cur = goal
    path = [cur]
    while cur != start:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path


# ----------------------- Algorithms (generator style) -----------------------
# Each algorithm is a generator that yields tuples of (to_open, to_close, final_path_or_None)
# - `to_open`   : list of coordinates that have been added to OPEN/frontier in this step
# - `to_close`  : list of coordinates that have been popped/visited in this step (CLOSED)
# - `final_path`: a list of coordinates if goal reached; None otherwise


def bfs_solve(grid: List[List[int]], start: Coord, goal: Coord) -> Generator[Tuple[List[Coord], List[Coord], Optional[List[Coord]]], None, None]:
    q = deque([start])
    came_from: Dict[Coord, Coord] = {}
    visited = {start}

    while q:
        cur = q.popleft()
        yield ([], [cur], None)
        if cur == goal:
            path = reconstruct_path(came_from, start, goal)
            yield ([], [], path)
            return
        for nbr in neighbors_4(*cur):
            r, c = nbr
            if grid[r][c] != WALL_S and nbr not in visited:
                visited.add(nbr)
                came_from[nbr] = cur
                q.append(nbr)
                yield ([nbr], [], None)


def dijkstra_solve(grid: List[List[int]], start: Coord, goal: Coord) -> Generator[Tuple[List[Coord], List[Coord], Optional[List[Coord]]], None, None]:
    import heapq

    pq: List[Tuple[int, Coord]] = []
    heapq.heappush(pq, (0, start))
    came_from: Dict[Coord, Coord] = {}
    dist: Dict[Coord, int] = {start: 0}
    visited = set()

    while pq:
        cur_cost, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        visited.add(cur)
        yield ([], [cur], None)
        if cur == goal:
            path = reconstruct_path(came_from, start, goal)
            yield ([], [], path)
            return
        for nbr in neighbors_4(*cur):
            r, c = nbr
            if grid[r][c] == WALL_S:
                continue
            new_cost = cur_cost + 1  # uniform grid cost = 1
            if new_cost < dist.get(nbr, math.inf):
                dist[nbr] = new_cost
                came_from[nbr] = cur
                heapq.heappush(pq, (new_cost, nbr))
                yield ([nbr], [], None)


def astar_solve(grid: List[List[int]], start: Coord, goal: Coord) -> Generator[Tuple[List[Coord], List[Coord], Optional[List[Coord]]], None, None]:
    import heapq

    pq: List[Tuple[int, Coord]] = []
    g: Dict[Coord, int] = {start: 0}
    f_start = h_manhattan(start, goal)
    heapq.heappush(pq, (f_start, start))
    came_from: Dict[Coord, Coord] = {}
    visited = set()

    while pq:
        f_cur, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        visited.add(cur)
        yield ([], [cur], None)
        if cur == goal:
            path = reconstruct_path(came_from, start, goal)
            yield ([], [], path)
            return
        for nbr in neighbors_4(*cur):
            r, c = nbr
            if grid[r][c] == WALL_S:
                continue
            tentative_g = g[cur] + 1
            if tentative_g < g.get(nbr, math.inf):
                g[nbr] = tentative_g
                came_from[nbr] = cur
                f_nbr = tentative_g + h_manhattan(nbr, goal)
                heapq.heappush(pq, (f_nbr, nbr))
                yield ([nbr], [], None)


# ----------------------- Pygame UI -----------------------

def draw_text(surface: pygame.Surface, text: str, x: int, y: int, size: int = 18):
    font = pygame.font.SysFont("consolas", size)
    img = font.render(text, True, TEXT)
    surface.blit(img, (x, y))


def draw_grid(surface: pygame.Surface, grid: List[List[int]], start: Optional[Coord], end: Optional[Coord],
              open_set: set[Coord], closed_set: set[Coord], path: List[Coord]):
    for r in range(ROWS):
        for c in range(COLS):
            rect = pygame.Rect(c * CELL_W, r * CELL_H, CELL_W - MARGIN, CELL_H - MARGIN)
            state = grid[r][c]
            color = WHITE
            if state == WALL_S:
                color = WALL
            elif (r, c) == start:
                color = START
            elif (r, c) == end:
                color = END
            elif (r, c) in path:
                color = PATH
            elif (r, c) in closed_set:
                color = CLOSED
            elif (r, c) in open_set:
                color = OPEN
            else:
                color = WHITE
            pygame.draw.rect(surface, color, rect)


# ----------------------- Persistence -----------------------

def save_grid(grid: List[List[int]], start: Optional[Coord], end: Optional[Coord], path: str = "grid.json"):
    data = {
        "rows": ROWS,
        "cols": COLS,
        "grid": grid,
        "start": start,
        "end": end,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f)


def load_grid(path: str = "grid.json") -> Tuple[List[List[int]], Optional[Coord], Optional[Coord]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if data["rows"] != ROWS or data["cols"] != COLS:
        raise ValueError("Saved grid size does not match current ROWS/COLS.")
    return data["grid"], tuple(data["start"]) if data["start"] else None, tuple(data["end"]) if data["end"] else None


# ----------------------- Random Maze -----------------------

def random_maze(density: float = 0.28) -> List[List[int]]:
    grid = [[EMPTY for _ in range(COLS)] for _ in range(ROWS)]
    for r in range(ROWS):
        for c in range(COLS):
            if random.random() < density:
                grid[r][c] = WALL_S
    return grid


# ----------------------- Main App -----------------------

def run():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Maze Solver: BFS / Dijkstra / A*")
    clock = pygame.time.Clock()

    grid = [[EMPTY for _ in range(COLS)] for _ in range(ROWS)]
    start: Optional[Coord] = None
    end: Optional[Coord] = None

    open_set: set[Coord] = set()
    closed_set: set[Coord] = set()
    path_draw: List[Coord] = []

    solver_gen: Optional[Generator] = None
    running = True
    fps = SPEED_FPS

    def reset_search_state():
        nonlocal open_set, closed_set, path_draw, solver_gen
        open_set.clear()
        closed_set.clear()
        path_draw = []
        solver_gen = None

    def start_solver(mode: str):
        nonlocal solver_gen
        if start is None or end is None:
            return
        reset_search_state()
        if mode == "bfs":
            solver_gen = bfs_solve(grid, start, end)
        elif mode == "dij":
            solver_gen = dijkstra_solve(grid, start, end)
        elif mode == "astar":
            solver_gen = astar_solve(grid, start, end)

    dragging_wall = False
    removing_wall = False

    while running:
        clock.tick(fps)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_b:
                    start_solver("bfs")
                elif event.key == pygame.K_d:
                    start_solver("dij")
                elif event.key == pygame.K_a:
                    start_solver("astar")
                elif event.key == pygame.K_c:
                    # clear search (keep walls/start/end)
                    reset_search_state()
                elif event.key == pygame.K_r:
                    # full reset
                    grid = [[EMPTY for _ in range(COLS)] for _ in range(ROWS)]
                    start = None
                    end = None
                    reset_search_state()
                elif event.key == pygame.K_g:
                    grid = random_maze()
                    reset_search_state()
                elif event.key == pygame.K_s:
                    try:
                        save_grid(grid, start, end)
                    except Exception as e:
                        print("Save failed:", e)
                elif event.key == pygame.K_l:
                    try:
                        grid, start, end = load_grid()
                        reset_search_state()
                    except Exception as e:
                        print("Load failed:", e)
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    fps = min(240, fps + 10)
                elif event.key == pygame.K_MINUS:
                    fps = max(10, fps - 10)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                r = event.pos[1] // CELL_H
                c = event.pos[0] // CELL_W
                if not in_bounds(r, c):
                    continue
                if event.button == 1:  # left click: toggle wall
                    if grid[r][c] == WALL_S:
                        grid[r][c] = EMPTY
                        removing_wall = True
                        dragging_wall = False
                    elif (r, c) != start and (r, c) != end:
                        grid[r][c] = WALL_S
                        dragging_wall = True
                        removing_wall = False
                    reset_search_state()
                elif event.button == 3:  # right click: set start/end
                    if start is None:
                        start = (r, c)
                    elif end is None and (r, c) != start:
                        end = (r, c)
                    else:
                        # cycle: move start, then end
                        if (r, c) == start:
                            start = None
                        elif (r, c) == end:
                            end = None
                    reset_search_state()

            elif event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:
                    r = event.pos[1] // CELL_H
                    c = event.pos[0] // CELL_W
                    if in_bounds(r, c) and (r, c) != start and (r, c) != end:
                        if dragging_wall:
                            grid[r][c] = WALL_S
                        elif removing_wall:
                            grid[r][c] = EMPTY

            elif event.type == pygame.MOUSEBUTTONUP:
                dragging_wall = False
                removing_wall = False

        # Step the solver if running
        if solver_gen is not None:
            try:
                to_open, to_close, final_path = next(solver_gen)
                for cell in to_open:
                    if cell != start and cell != end and grid[cell[0]][cell[1]] == EMPTY:
                        open_set.add(cell)
                for cell in to_close:
                    if cell != start and cell != end:
                        open_set.discard(cell)
                        closed_set.add(cell)
                if final_path is not None:
                    path_draw = [p for p in final_path if p not in (start, end)]
                    solver_gen = None
            except StopIteration:
                solver_gen = None

        # Draw
        screen.fill(BLACK)
        draw_grid(screen, grid, start, end, open_set, closed_set, path_draw)
        # Overlay text
        draw_text(screen, "B= BFS  D= Dijkstra  A= A*   C= Clear search  R= Reset  G= Random  S/L= Save/Load  +/-=Speed", 10, 10)
        if start:
            draw_text(screen, f"Start: {start}", 10, 34)
        if end:
            draw_text(screen, f"End:   {end}", 10, 54)
        if path_draw:
            draw_text(screen, f"Path length: {len(path_draw)+1 if start and end else len(path_draw)}", 10, 74)
        draw_text(screen, f"FPS: {fps}", 10, 94)

        pygame.display.flip()

    pygame.quit()


# ----------------------- Tests (CLI) -----------------------
# Run with: python maze_solver.py --test

def _solve_collect(gen: Generator):
    """Consume a solver generator and return the final path (or None)."""
    final_path = None
    for to_open, to_close, maybe_path in gen:
        if maybe_path is not None:
            final_path = maybe_path
            break
    return final_path


def run_tests():
    # 1) Trivial 3x3 open grid shortest path
    small_rows, small_cols = 3, 3
    g = [[EMPTY for _ in range(small_cols)] for _ in range(small_rows)]

    def nb(r, c):
        return [(r+dr, c+dc) for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
                if 0 <= r+dr < small_rows and 0 <= c+dc < small_cols]

    # Temporarily monkey-patch neighbors_4 and global dimensions for tests
    global ROWS, COLS
    old_rows, old_cols = ROWS, COLS
    ROWS, COLS = small_rows, small_cols

    start, goal = (0, 0), (2, 2)

    # BFS
    path_bfs = _solve_collect(bfs_solve(g, start, goal))
    assert path_bfs is not None, "BFS should find a path on open grid"

    # Dijkstra
    path_dij = _solve_collect(dijkstra_solve(g, start, goal))
    assert path_dij is not None, "Dijkstra should find a path on open grid"

    # A*
    path_astar = _solve_collect(astar_solve(g, start, goal))
    assert path_astar is not None, "A* should find a path on open grid"

    # All should yield the same shortest length on unit grid
    Lb, Ld, La = len(path_bfs), len(path_dij), len(path_astar)
    assert Lb == Ld == La, f"Path lengths differ: BFS={Lb}, Dij={Ld}, A*={La}"

    # 2) Blocked goal -> no path
    g_block = [[EMPTY for _ in range(small_cols)] for _ in range(small_rows)]
    g_block[1][2] = WALL_S
    g_block[2][1] = WALL_S

    no_path = _solve_collect(bfs_solve(g_block, start, goal))
    assert no_path is None, "BFS should return None when goal is blocked"

    no_path = _solve_collect(dijkstra_solve(g_block, start, goal))
    assert no_path is None, "Dijkstra should return None when goal is blocked"

    no_path = _solve_collect(astar_solve(g_block, start, goal))
    assert no_path is None, "A* should return None when goal is blocked"

    # 3) Narrow corridor ensures unique shortest path
    g_corr = [
        [EMPTY, WALL_S, EMPTY],
        [EMPTY, WALL_S, EMPTY],
        [EMPTY, EMPTY, EMPTY],
    ]
    start, goal = (0, 0), (2, 2)

    pb = _solve_collect(bfs_solve(g_corr, start, goal))
    pd = _solve_collect(dijkstra_solve(g_corr, start, goal))
    pa = _solve_collect(astar_solve(g_corr, start, goal))
    assert pb and pd and pa
    assert len(pb) == len(pd) == len(pa)

    # restore globals
    ROWS, COLS = old_rows, old_cols

    print("All tests passed.")


if __name__ == "__main__":
    if "--test" in sys.argv:
        run_tests()
        sys.exit(0)
    try:
        run()
    except KeyboardInterrupt:
        pygame.quit()
        sys.exit(0)
