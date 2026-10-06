/**
 * @file maze_solver.c
 * @brief Maze solver with Persistent Map + Flood Fill + A*
 *
 * Architecture:
 *   Maze_Persistent_t  — accumulated wall knowledge across ALL runs
 *   Maze_t             — per-run working data (reset each explore)
 *   Maze_AStarResult_t — computed optimal path for speed runs
 */

#include "maze_solver.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define MAX_DISTANCE 0xFFFF

static const int8_t dx[4] = {0, 1, 0, -1};
static const int8_t dy[4] = {1, 0, -1, 0};

static uint8_t IsValidCell(int8_t x, int8_t y) {
  return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

static uint8_t HasWall(Maze_t *maze, uint8_t x, uint8_t y, uint8_t dir) {
  return (maze->walls[y][x] & (1 << dir)) != 0;
}

static void SetWall(Maze_t *maze, uint8_t x, uint8_t y, uint8_t dir) {
  int8_t nx, ny;
  uint8_t opposite_dir;

  maze->walls[y][x] |= (1 << dir);

  nx = x + dx[dir];
  ny = y + dy[dir];

  if (IsValidCell(nx, ny)) {
    opposite_dir = (dir + 2) % 4;
    maze->walls[ny][nx] |= (1 << opposite_dir);
  }
}

static void ClearWall(Maze_t *maze, uint8_t x, uint8_t y, uint8_t dir) {
  int8_t nx, ny;
  uint8_t opposite_dir;

  maze->walls[y][x] &= ~(1 << dir);

  nx = x + dx[dir];
  ny = y + dy[dir];

  if (IsValidCell(nx, ny)) {
    opposite_dir = (dir + 2) % 4;
    maze->walls[ny][nx] &= ~(1 << opposite_dir);
  }
}

static void FloodFillCore(Maze_t *maze, uint8_t target_x, uint8_t target_y) {
  static uint8_t queue_x[MAZE_SIZE * MAZE_SIZE];
  static uint8_t queue_y[MAZE_SIZE * MAZE_SIZE];
  uint16_t head = 0;
  uint16_t tail = 0;
  uint8_t x, y, dir;
  int8_t nx, ny;
  uint16_t new_dist;

  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      maze->flood[y][x] = MAX_DISTANCE;
    }
  }

  maze->flood[target_y][target_x] = 0;
  queue_x[tail] = target_x;
  queue_y[tail] = target_y;
  tail++;

  while (head != tail) {
    x = queue_x[head];
    y = queue_y[head];
    head++;

    for (dir = 0; dir < 4; dir++) {
      if (HasWall(maze, x, y, dir)) {
        continue;
      }

      nx = x + dx[dir];
      ny = y + dy[dir];

      if (!IsValidCell(nx, ny)) {
        continue;
      }

      new_dist = maze->flood[y][x] + 1;

      if (new_dist < maze->flood[ny][nx]) {
        maze->flood[ny][nx] = new_dist;
        queue_x[tail] = nx;
        queue_y[tail] = ny;
        tail++;
      }
    }
  }
}

static uint8_t FindBestDirection(Maze_t *maze) {
  uint8_t x = maze->robot_x;
  uint8_t y = maze->robot_y;
  uint8_t best_dir = 0;
  uint16_t min_dist = MAX_DISTANCE;
  uint8_t dir;
  int8_t nx, ny;

  for (dir = 0; dir < 4; dir++) {
    if (HasWall(maze, x, y, dir)) {
      continue;
    }

    nx = x + dx[dir];
    ny = y + dy[dir];

    if (!IsValidCell(nx, ny)) {
      continue;
    }

    if (maze->flood[ny][nx] < min_dist) {
      min_dist = maze->flood[ny][nx];
      best_dir = dir;
    }
  }

  return best_dir;
}

static uint8_t CalculateTurnCommand(uint8_t current_dir, uint8_t target_dir) {
  int8_t diff = (int8_t)target_dir - (int8_t)current_dir;

  if (diff < -2)
    diff += 4;
  if (diff > 2)
    diff -= 4;

  switch (diff) {
  case 0:
    return 'F';
  case 1:
    return 'R';
  case -1:
    return 'L';
  case 2:
  case -2:
    return 'B';
  default:
    return 'F';
  }
}

void Maze_Init(Maze_t *maze) {
  uint8_t x, y;

  memset(maze, 0, sizeof(Maze_t));

  maze->robot_x = 0;
  maze->robot_y = 0;
  maze->robot_dir = 0;

  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      maze->walls[y][x] = 0;
      maze->visited[y][x] = 0;
      maze->flood[y][x] = MAX_DISTANCE;
    }
  }

  for (x = 0; x < MAZE_SIZE; x++) {
    SetWall(maze, x, 0, 2);
    SetWall(maze, x, MAZE_SIZE - 1, 0);
  }

  for (y = 0; y < MAZE_SIZE; y++) {
    SetWall(maze, 0, y, 3);
    SetWall(maze, MAZE_SIZE - 1, y, 1);
  }

  maze->visited[0][0] = 1;

  maze->persistent_ref = (void *)0; /* NULL — no persistent protection by default */
  maze->path_len = 0;
}

void Maze_UpdateWalls(Maze_t *maze, uint8_t left, uint8_t front,
                      uint8_t right) {
  uint8_t x = maze->robot_x;
  uint8_t y = maze->robot_y;
  uint8_t dir = maze->robot_dir;
  uint8_t front_dir, left_dir, right_dir;
  uint8_t conf = 0;

  front_dir = dir;
  left_dir = (dir + 3) % 4;
  right_dir = (dir + 1) % 4;

  /* If persistent map is linked, get the confidence bitfield for this cell.
   * Any direction already observed in persistent map is immutable. */
  if (maze->persistent_ref != (void *)0) {
    conf = maze->persistent_ref->confidence[y][x];
  }

#if MAZE_WALL_ADD_ONLY
  /* Add-only mode: only SET newly detected walls, never clear existing ones.
   * Skip directions already observed in persistent map. */
  if (!(conf & (1 << front_dir))) {
    if (front) SetWall(maze, x, y, front_dir);
  }
  if (!(conf & (1 << left_dir))) {
    if (left)  SetWall(maze, x, y, left_dir);
  }
  if (!(conf & (1 << right_dir))) {
    if (right) SetWall(maze, x, y, right_dir);
  }
#else
  /* Full update: SET detected walls, CLEAR undetected walls.
   * No persistent protection here — always update to latest sensor data. */
  if (front) SetWall(maze, x, y, front_dir);
  else       ClearWall(maze, x, y, front_dir);

  if (left)  SetWall(maze, x, y, left_dir);
  else       ClearWall(maze, x, y, left_dir);

  if (right) SetWall(maze, x, y, right_dir);
  else       ClearWall(maze, x, y, right_dir);
#endif

  maze->visited[y][x] = 1;
}

/* Add-only variant: never clears existing walls, only adds newly detected ones.
 * Used by secondary sensor reads (POST_TURN, POST_ALIGN, POST_BA). */
void Maze_AddWalls(Maze_t *maze, uint8_t left, uint8_t front, uint8_t right) {
  uint8_t x = maze->robot_x;
  uint8_t y = maze->robot_y;
  uint8_t dir = maze->robot_dir;

  if (front) SetWall(maze, x, y, dir);
  if (left)  SetWall(maze, x, y, (dir + 3) % 4);
  if (right) SetWall(maze, x, y, (dir + 1) % 4);
}

void Maze_FloodFill(Maze_t *maze, uint8_t target_x, uint8_t target_y) {
  FloodFillCore(maze, target_x, target_y);
}

void Maze_FloodFillCenter(Maze_t *maze) {
  static uint8_t queue_x[MAZE_SIZE * MAZE_SIZE];
  static uint8_t queue_y[MAZE_SIZE * MAZE_SIZE];
  uint16_t head = 0;
  uint16_t tail = 0;
  uint8_t x, y, dir, tx, ty;
  int8_t nx, ny;
  uint16_t new_dist;

  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      maze->flood[y][x] = MAX_DISTANCE;
    }
  }

  /* Seed all 4 center cells with distance 0 */
  for (ty = MAZE_CENTER_Y1; ty <= MAZE_CENTER_Y2; ty++) {
    for (tx = MAZE_CENTER_X1; tx <= MAZE_CENTER_X2; tx++) {
      maze->flood[ty][tx] = 0;
      queue_x[tail] = tx;
      queue_y[tail] = ty;
      tail++;
    }
  }

  while (head != tail) {
    x = queue_x[head];
    y = queue_y[head];
    head++;

    for (dir = 0; dir < 4; dir++) {
      if (HasWall(maze, x, y, dir))
        continue;

      nx = x + dx[dir];
      ny = y + dy[dir];

      if (!IsValidCell(nx, ny))
        continue;

      new_dist = maze->flood[y][x] + 1;
      if (new_dist < maze->flood[ny][nx]) {
        maze->flood[ny][nx] = new_dist;
        queue_x[tail] = nx;
        queue_y[tail] = ny;
        tail++;
      }
    }
  }
}

uint8_t Maze_GetNextMove(Maze_t *maze) {
  uint8_t best_dir;
  uint8_t command;

  best_dir = FindBestDirection(maze);

  command = CalculateTurnCommand(maze->robot_dir, best_dir);

  /* NOTE: Do NOT update robot_x/y/dir here.
   * Position tracking is handled by the caller (Execute_MazeExplore)
   * to avoid double-update bugs that cause out-of-bounds access. */

  return command;
}

void Maze_OptimizePath(Maze_t *maze) {
  uint8_t i, j;
  uint8_t optimized_path[256];
  uint8_t opt_len = 0;
  uint8_t count;

  if (maze->path_len == 0)
    return;

  i = 0;
  while (i < maze->path_len) {
    if (maze->path[i] == 'F') {
      count = 0;
      j = i;
      while (j < maze->path_len && maze->path[j] == 'F') {
        count++;
        j++;
      }

      optimized_path[opt_len++] = count;
      i = j;
    } else {
      optimized_path[opt_len++] = maze->path[i];
      i++;
    }
  }

  for (i = 0; i < opt_len && i < 256; i++) {
    maze->path[i] = optimized_path[i];
  }
  maze->path_len = opt_len;
}

uint8_t Maze_IsAtTarget(Maze_t *maze, uint8_t target_x, uint8_t target_y) {
  return (maze->robot_x == target_x && maze->robot_y == target_y);
}

uint8_t Maze_IsAtCenter(Maze_t *maze) {
  return IS_CENTER_CELL(maze->robot_x, maze->robot_y);
}

void Maze_ResetPosition(Maze_t *maze) {
  maze->robot_x = 0;
  maze->robot_y = 0;
  maze->robot_dir = 0;
}

uint16_t Maze_GetDistanceToTarget(Maze_t *maze) {
  return maze->flood[maze->robot_y][maze->robot_x];
}

void Maze_Print(Maze_t *maze) {
  uint8_t x, y;
  char line[128];
  uint8_t pos;

  UART_SendString("\r\n=== MAZE MAP ===\r\n");
  delay_ms_blocking(10);

  for (y = MAZE_SIZE; y > 0; y--) {
    /* Top walls */
    pos = 0;
    for (x = 0; x < MAZE_SIZE; x++) {
      line[pos++] = '+';
      if (HasWall(maze, x, y - 1, 0)) {
        line[pos++] = '-';
        line[pos++] = '-';
        line[pos++] = '-';
      } else {
        line[pos++] = ' ';
        line[pos++] = ' ';
        line[pos++] = ' ';
      }
    }
    line[pos++] = '+';
    line[pos++] = '\r';
    line[pos++] = '\n';
    line[pos] = '\0';
    UART_SendString(line);
    delay_ms_blocking(5);

    /* Cell contents */
    pos = 0;
    for (x = 0; x < MAZE_SIZE; x++) {
      if (HasWall(maze, x, y - 1, 3)) {
        line[pos++] = '|';
      } else {
        line[pos++] = ' ';
      }

      if (maze->robot_x == x && maze->robot_y == y - 1) {
        line[pos++] = ' ';
        line[pos++] = '*';
        line[pos++] = ' ';
      } else if (maze->visited[y - 1][x]) {
        uint16_t dist = maze->flood[y - 1][x];
        if (dist > 99) {
          line[pos++] = '9';
          line[pos++] = '9';
          line[pos++] = '+';
        } else {
          char d_buf[4];
          sprintf(d_buf, "%2d", dist);
          line[pos++] = ' ';
          line[pos++] = d_buf[0];
          line[pos++] = d_buf[1];
        }
      } else {
        line[pos++] = ' ';
        line[pos++] = '.';
        line[pos++] = ' ';
      }
    }
    line[pos++] = '|';
    line[pos++] = '\r';
    line[pos++] = '\n';
    line[pos] = '\0';
    UART_SendString(line);
    delay_ms_blocking(5);
  }

  /* Bottom walls */
  pos = 0;
  for (x = 0; x < MAZE_SIZE; x++) {
    line[pos++] = '+';
    if (HasWall(maze, x, 0, 2)) {
      line[pos++] = '-';
      line[pos++] = '-';
      line[pos++] = '-';
    } else {
      line[pos++] = ' ';
      line[pos++] = ' ';
      line[pos++] = ' ';
    }
  }
  line[pos++] = '+';
  line[pos++] = '\r';
  line[pos++] = '\n';
  line[pos] = '\0';
  UART_SendString(line);
}

/* ========================================================================== */
/* COMPACT MAZE DUMP — for BLE web visualization                              */
/*   Format: MAZE:RX,RY,RD:WALLS_256H:FLOOD_512H\r\n                         */
/*   WALLS: 256 hex chars (1 per cell), FLOOD: 512 hex chars (2 per cell)    */
/* ========================================================================== */
void Maze_PrintCompact(Maze_t *maze) {
  static const char hex[] = "0123456789ABCDEF";
  /* [FIX] Split into 8 small frames (~80 bytes each) to avoid BLE overflow.
   * Protocol:
   *   MZ0:RX,RY,RD:WALLS_ROW_0-3   (header + 64 wall hex chars)
   *   MZ1:WALLS_ROW_4-7             (64 wall hex chars)
   *   MZ2:WALLS_ROW_8-11            (64 wall hex chars)
   *   MZ3:WALLS_ROW_12-15           (64 wall hex chars)
   *   MZ4:FLOOD_ROW_0-3             (128 flood hex chars)
   *   MZ5:FLOOD_ROW_4-7             (128 flood hex chars)
   *   MZ6:FLOOD_ROW_8-11            (128 flood hex chars)
   *   MZ7:FLOOD_ROW_12-15           (128 flood hex chars)
   * Web console reassembles from MZ0..MZ7 before rendering. */
  char buf[160];
  uint8_t x, y, chunk;
  uint16_t pos, fv;

  for (chunk = 0; chunk < 8; chunk++) {
    pos = 0;
    if (chunk == 0) {
      /* MZ0 includes robot position */
      pos = sprintf(buf, "MZ0:%d,%d,%d:",
                    maze->robot_x, maze->robot_y, maze->robot_dir);
    } else {
      pos = sprintf(buf, "MZ%d:", chunk);
    }

    if (chunk < 4) {
      /* Walls: 4 rows per chunk = 64 hex chars */
      uint8_t y_start = chunk * 4;
      for (y = y_start; y < y_start + 4; y++) {
        for (x = 0; x < MAZE_SIZE; x++) {
          buf[pos++] = hex[maze->walls[y][x] & 0x0F];
        }
      }
    } else {
      /* Flood: 4 rows per chunk = 128 flood hex chars */
      uint8_t y_start = (chunk - 4) * 4;
      for (y = y_start; y < y_start + 4; y++) {
        for (x = 0; x < MAZE_SIZE; x++) {
          fv = maze->flood[y][x];
          if (fv > 255) fv = 255;
          buf[pos++] = hex[(fv >> 4) & 0x0F];
          buf[pos++] = hex[fv & 0x0F];
        }
      }
    }
    buf[pos++] = '\r';
    buf[pos++] = '\n';
    buf[pos] = '\0';
    UART_SendString(buf);
    delay_ms_blocking(80);
  }
}

/* ========================================================================== */
/* SINGLE CELL UPDATE — sent during explore/A* after each Maze_UpdateWalls    */
/*   Format: CELL:X,Y,W,RD,FF\r\n                                            */
/*   W = walls hex, FF = flood hex (2 chars)                                  */
/* ========================================================================== */
void Maze_SendCellUpdate(Maze_t *maze) {
  static const char hex[] = "0123456789ABCDEF";
  char buf[32];
  uint8_t x = maze->robot_x;
  uint8_t y = maze->robot_y;
  uint16_t fv = maze->flood[y][x];
  if (fv > 255) fv = 255;

  sprintf(buf, "CELL:%d,%d,%c,%d,%c%c\r\n", x, y,
          hex[maze->walls[y][x] & 0x0F], maze->robot_dir,
          hex[(fv >> 4) & 0x0F], hex[fv & 0x0F]);
  UART_SendString(buf);
}

/* ========================================================================== */
/* PERSISTENT MAP IMPLEMENTATION                                              */
/* ========================================================================== */

static void SetWallRaw(uint8_t walls[][MAZE_SIZE], uint8_t x, uint8_t y,
                       uint8_t dir) {
  int8_t nx, ny;
  uint8_t opposite_dir;

  walls[y][x] |= (1 << dir);

  nx = x + dx[dir];
  ny = y + dy[dir];

  if (IsValidCell(nx, ny)) {
    opposite_dir = (dir + 2) % 4;
    walls[ny][nx] |= (1 << opposite_dir);
  }
}

void Maze_Persistent_Init(Maze_Persistent_t *pm) {
  /* Only init if not already valid (preserves data across soft resets) */
  if (pm->is_valid == MAZE_PERSISTENT_MAGIC) {
    UART_SendString("Persistent map: LOADED (existing data)\r\n");
    return;
  }

  Maze_Persistent_Reset(pm);
  UART_SendString("Persistent map: INITIALIZED (fresh)\r\n");
}

void Maze_Persistent_Reset(Maze_Persistent_t *pm) {
  uint8_t x, y;

  memset(pm, 0, sizeof(Maze_Persistent_t));

  /* Set boundary walls */
  for (x = 0; x < MAZE_SIZE; x++) {
    SetWallRaw(pm->walls, x, 0, 2);             /* South boundary */
    SetWallRaw(pm->walls, x, MAZE_SIZE - 1, 0); /* North boundary */
  }
  for (y = 0; y < MAZE_SIZE; y++) {
    SetWallRaw(pm->walls, 0, y, 3);             /* West boundary  */
    SetWallRaw(pm->walls, MAZE_SIZE - 1, y, 1); /* East boundary  */
  }

  pm->total_runs = 0;
  pm->center_reached_count = 0;
  pm->fully_explored = 0;
  pm->is_valid = MAZE_PERSISTENT_MAGIC;

  UART_SendString("Persistent map: RESET\r\n");
}

void Maze_Persistent_LoadInto(Maze_Persistent_t *pm, Maze_t *maze) {
  uint8_t x, y;

  /* Copy persistent walls into per-run maze */
  memcpy(maze->walls, pm->walls, sizeof(maze->walls));

  /* Reset per-run data */
  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      maze->flood[y][x] = MAX_DISTANCE;
      maze->visited[y][x] = 0;
    }
  }

  maze->robot_x = 0;
  maze->robot_y = 0;
  maze->robot_dir = 0;
  maze->visited[0][0] = 1;
  maze->path_len = 0;

  /* Link persistent map so Maze_UpdateWalls can check confidence */
  maze->persistent_ref = pm;
}

void Maze_Persistent_SyncFrom(Maze_Persistent_t *pm, Maze_t *maze) {
  uint8_t x, y;

  /* Copy all walls from per-run maze back to persistent.
   * This includes both old knowledge and newly discovered walls. */
  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      pm->walls[y][x] = maze->walls[y][x];
    }
  }
}

void Maze_Persistent_MarkObserved(Maze_Persistent_t *pm, uint8_t x, uint8_t y,
                                  uint8_t robot_dir) {
  uint8_t front_dir, left_dir, right_dir;

  if (!IsValidCell(x, y))
    return;

  front_dir = robot_dir;
  left_dir = (robot_dir + 3) % 4;
  right_dir = (robot_dir + 1) % 4;

  /* Mark that we observed front/left/right walls from this cell */
  pm->confidence[y][x] |= (1 << front_dir);
  pm->confidence[y][x] |= (1 << left_dir);
  pm->confidence[y][x] |= (1 << right_dir);
}

void Maze_Persistent_IncrementVisit(Maze_Persistent_t *pm, uint8_t x,
                                    uint8_t y) {
  if (!IsValidCell(x, y))
    return;
  if (pm->visit_count[y][x] < 255) {
    pm->visit_count[y][x]++;
  }
}

uint8_t Maze_Persistent_GetCoverage(Maze_Persistent_t *pm) {
  uint8_t x, y;
  uint16_t observed = 0;
  uint16_t total = MAZE_SIZE * MAZE_SIZE;

  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      if (pm->confidence[y][x] != 0) {
        observed++;
      }
    }
  }

  return (uint8_t)((observed * 100) / total);
}

uint8_t Maze_Persistent_IsReadyForAStar(Maze_Persistent_t *pm) {
  /* Need at least 1 successful exploration to center */
  return (pm->is_valid == MAZE_PERSISTENT_MAGIC &&
          pm->center_reached_count > 0);
}

void Maze_Persistent_PrintStatus(Maze_Persistent_t *pm) {
  char buf[128];

  UART_SendString("\r\n=== PERSISTENT MAP STATUS ===\r\n");
  sprintf(buf, "  Valid: %s\r\n",
          (pm->is_valid == MAZE_PERSISTENT_MAGIC) ? "YES" : "NO");
  UART_SendString(buf);
  sprintf(buf, "  Total runs: %d\r\n", pm->total_runs);
  UART_SendString(buf);
  sprintf(buf, "  Center reached: %d times\r\n", pm->center_reached_count);
  UART_SendString(buf);
  sprintf(buf, "  Coverage: %d%%\r\n", Maze_Persistent_GetCoverage(pm));
  UART_SendString(buf);
  sprintf(buf, "  A* ready: %s\r\n",
          Maze_Persistent_IsReadyForAStar(pm) ? "YES" : "NO");
  UART_SendString(buf);
}

/* ========================================================================== */
/* SMART EXPLORATION                                                          */
/* ========================================================================== */

/** Find best direction using persistent visit_count to break flood-fill ties */
static uint8_t FindBestDirection_Smart(Maze_t *maze, Maze_Persistent_t *pm) {
  uint8_t x = maze->robot_x;
  uint8_t y = maze->robot_y;
  uint8_t best_dir = 0;
  uint16_t min_score = 0xFFFF;
  uint8_t dir;
  int8_t nx, ny;
  uint16_t score;

  for (dir = 0; dir < 4; dir++) {
    if (HasWall(maze, x, y, dir)) {
      continue;
    }

    nx = x + dx[dir];
    ny = y + dy[dir];

    if (!IsValidCell(nx, ny)) {
      continue;
    }

    /* Base score = flood distance to target */
    score = maze->flood[ny][nx];

    /* Penalize cells visited many times across ALL runs */
    if (pm->visit_count[ny][nx] > 0) {
      score += VISITED_CELL_PENALTY * pm->visit_count[ny][nx];
    }

    /* Extra penalty for cells visited in THIS run */
    if (maze->visited[ny][nx]) {
      score += VISITED_CELL_PENALTY;
    }

    /* Bonus: favor cells with low confidence (less observed) */
    {
      uint8_t conf = pm->confidence[ny][nx];
      uint8_t observed_dirs = 0;
      uint8_t d;
      for (d = 0; d < 4; d++) {
        if (conf & (1 << d))
          observed_dirs++;
      }
      /* Cells with fewer observed directions get priority */
      if (observed_dirs < 4 && score > (4 - observed_dirs) * 10) {
        score -= (4 - observed_dirs) * 10;
      }
    }

    if (score < min_score) {
      min_score = score;
      best_dir = dir;
    }
  }

  return best_dir;
}

uint8_t Maze_GetNextMove_Smart(Maze_t *maze, Maze_Persistent_t *pm) {
  uint8_t best_dir;
  uint8_t command;
  uint8_t x = maze->robot_x;
  uint8_t y = maze->robot_y;
  uint8_t dir;
  uint8_t has_open_path = 0;

  /* [FIX] Safety: check if any direction is open before computing */
  for (dir = 0; dir < 4; dir++) {
    if (!HasWall(maze, x, y, dir)) {
      int8_t nx = x + dx[dir];
      int8_t ny = y + dy[dir];
      if (IsValidCell(nx, ny)) {
        has_open_path = 1;
        break;
      }
    }
  }

  if (!has_open_path) {
    /* All directions walled off - should never happen in valid maze.
     * Return B-turn to trigger re-evaluation after direction change. */
    UART_SendString("WARN: All walls closed! Returning B-turn\r\n");
    return 'B';
  }

  best_dir = FindBestDirection_Smart(maze, pm);
  command = CalculateTurnCommand(maze->robot_dir, best_dir);

  /* NOTE: Do NOT update robot_x/y/dir here. Caller handles it. */

  return command;
}

/* ========================================================================== */
/* A* OPTIMAL PATH                                                            */
/* ========================================================================== */

static uint16_t AStar_Heuristic(uint8_t x, uint8_t y) {
  /* Manhattan distance to nearest cell in center region */
  uint16_t dx_val = 0, dy_val = 0;
  if (x < MAZE_CENTER_X1)
    dx_val = MAZE_CENTER_X1 - x;
  else if (x > MAZE_CENTER_X2)
    dx_val = x - MAZE_CENTER_X2;
  if (y < MAZE_CENTER_Y1)
    dy_val = MAZE_CENTER_Y1 - y;
  else if (y > MAZE_CENTER_Y2)
    dy_val = y - MAZE_CENTER_Y2;
  return dx_val + dy_val;
}

uint8_t Maze_CalculateOptimalPath(Maze_t *maze, Maze_AStarResult_t *result) {
  /* [CRITICAL FIX] These arrays were on the stack (~3.2KB) causing stack
   * overflow on STM32F411 (default stack 1-2KB).  Moving to static (BSS)
   * fixes the HardFault that froze the system in A* mode. */
  static AStarNode_t open_list[1024];
  uint16_t open_count = 0;

  static uint16_t closed_cost[MAZE_SIZE][MAZE_SIZE][4];
  static AStarNode_t parent_map[MAZE_SIZE][MAZE_SIZE][4];

  uint8_t x, y, dir;
  uint16_t i;
  AStarNode_t current, neighbor;
  uint16_t tentative_g;
  uint8_t found_goal = 0;
  AStarNode_t goal_node;
  AStarNode_t start;
  char buf[80];
  uint16_t iterations = 0;

  const uint16_t COST_FORWARD  = 100;
  const uint16_t COST_TURN_90  = 220;
  const uint16_t COST_TURN_180 = 450;
  const uint16_t MAX_ITERATIONS = 8000; /* Safety limit (increased for 16x16) */

  for (y = 0; y < MAZE_SIZE; y++) {
    for (x = 0; x < MAZE_SIZE; x++) {
      for (dir = 0; dir < 4; dir++) {
        closed_cost[y][x][dir] = 0xFFFF;
        parent_map[y][x][dir].command = 0;
      }
    }
  }

  start.x = maze->robot_x;
  start.y = maze->robot_y;
  start.direction = maze->robot_dir;
  start.g_cost = 0;
  start.f_cost = AStar_Heuristic(start.x, start.y);
  start.command = 0;

  open_list[open_count++] = start;
  closed_cost[start.y][start.x][start.direction] = 0;

  while (open_count > 0 && iterations < MAX_ITERATIONS) {
    uint8_t min_idx = 0;
    iterations++;

    for (i = 1; i < open_count; i++) {
      if (open_list[i].f_cost < open_list[min_idx].f_cost) {
        min_idx = i;
      }
    }

    current = open_list[min_idx];

    for (i = min_idx; i < open_count - 1; i++) {
      open_list[i] = open_list[i + 1];
    }
    open_count--;

    if (IS_CENTER_CELL(current.x, current.y)) {
      found_goal = 1;
      goal_node = current;
      break;
    }

    /* Try FORWARD */
    if (!HasWall(maze, current.x, current.y, current.direction)) {
      neighbor.x = current.x + dx[current.direction];
      neighbor.y = current.y + dy[current.direction];
      neighbor.direction = current.direction;

      if (IsValidCell(neighbor.x, neighbor.y)) {
        tentative_g = current.g_cost + COST_FORWARD;

        if (tentative_g <
            closed_cost[neighbor.y][neighbor.x][neighbor.direction]) {
          closed_cost[neighbor.y][neighbor.x][neighbor.direction] = tentative_g;
          neighbor.g_cost = tentative_g;
          neighbor.f_cost =
              tentative_g + AStar_Heuristic(neighbor.x, neighbor.y);

          parent_map[neighbor.y][neighbor.x][neighbor.direction] = current;
          parent_map[neighbor.y][neighbor.x][neighbor.direction].command = 'F';

          if (open_count < 1024) {
            open_list[open_count++] = neighbor;
          }
        }
      }
    }

    /* Try LEFT turn */
    neighbor.x = current.x;
    neighbor.y = current.y;
    neighbor.direction = (current.direction + 3) % 4;
    tentative_g = current.g_cost + COST_TURN_90;

    if (tentative_g < closed_cost[neighbor.y][neighbor.x][neighbor.direction]) {
      closed_cost[neighbor.y][neighbor.x][neighbor.direction] = tentative_g;
      neighbor.g_cost = tentative_g;
      neighbor.f_cost = tentative_g + AStar_Heuristic(neighbor.x, neighbor.y);

      parent_map[neighbor.y][neighbor.x][neighbor.direction] = current;
      parent_map[neighbor.y][neighbor.x][neighbor.direction].command = 'L';

      if (open_count < 1024) {
        open_list[open_count++] = neighbor;
      }
    }

    /* Try RIGHT turn */
    neighbor.x = current.x;
    neighbor.y = current.y;
    neighbor.direction = (current.direction + 1) % 4;
    tentative_g = current.g_cost + COST_TURN_90;

    if (tentative_g < closed_cost[neighbor.y][neighbor.x][neighbor.direction]) {
      closed_cost[neighbor.y][neighbor.x][neighbor.direction] = tentative_g;
      neighbor.g_cost = tentative_g;
      neighbor.f_cost = tentative_g + AStar_Heuristic(neighbor.x, neighbor.y);

      parent_map[neighbor.y][neighbor.x][neighbor.direction] = current;
      parent_map[neighbor.y][neighbor.x][neighbor.direction].command = 'R';

      if (open_count < 1024) {
        open_list[open_count++] = neighbor;
      }
    }

    /* Try U-TURN */
    neighbor.x = current.x;
    neighbor.y = current.y;
    neighbor.direction = (current.direction + 2) % 4;
    tentative_g = current.g_cost + COST_TURN_180;

    if (tentative_g < closed_cost[neighbor.y][neighbor.x][neighbor.direction]) {
      closed_cost[neighbor.y][neighbor.x][neighbor.direction] = tentative_g;
      neighbor.g_cost = tentative_g;
      neighbor.f_cost = tentative_g + AStar_Heuristic(neighbor.x, neighbor.y);

      parent_map[neighbor.y][neighbor.x][neighbor.direction] = current;
      parent_map[neighbor.y][neighbor.x][neighbor.direction].command = 'B';

      if (open_count < 1024) {
        open_list[open_count++] = neighbor;
      }
    }
  }

  if (!found_goal) {
    sprintf(buf, "A* FAILED: iters=%u open=%u\r\n", iterations, open_count);
    UART_SendString(buf);
    result->optimal_path_length = 0;
    return 0;
  }

  sprintf(buf, "A* SUCCESS: Cost=%u iters=%u\r\n", goal_node.g_cost,
          iterations);
  UART_SendString(buf);

  /* Reconstruct path by walking parent_map backwards */
  result->optimal_path_length = 0;
  current = goal_node;

  {
    uint8_t max_retrace =
        MAZE_SIZE * MAZE_SIZE * 4; /* Max possible unique states */
    uint8_t retrace_count = 0;

    while (!(current.x == start.x && current.y == start.y &&
             current.direction == start.direction)) {
      AStarNode_t parent;

      /* [FIX] Bounds check to prevent out-of-bounds access */
      if (current.x >= MAZE_SIZE || current.y >= MAZE_SIZE ||
          current.direction >= 4) {
        UART_SendString("A* RETRACE: OOB!\r\n");
        break;
      }

      parent = parent_map[current.y][current.x][current.direction];

      if (parent.command == 0)
        break;

      /* [FIX] Cycle/runaway detection */
      retrace_count++;
      if (retrace_count > max_retrace) {
        UART_SendString("A* RETRACE: cycle detected!\r\n");
        break;
      }

      /* Shift array right to insert at front */
      for (i = result->optimal_path_length; i > 0; i--) {
        result->optimal_path[i] = result->optimal_path[i - 1];
      }

      result->optimal_path[0].command = parent.command;
      result->optimal_path[0].x = current.x;
      result->optimal_path[0].y = current.y;
      result->optimal_path[0].direction = current.direction;

      result->optimal_path_length++;

      if (result->optimal_path_length >= 255)
        break;

      current = parent;
    }
  }

  result->optimal_path_index = 0;

  sprintf(buf, "A* Path: %d actions\r\n", result->optimal_path_length);
  UART_SendString(buf);

  return 1;
}

uint8_t Maze_GetNextMove_Optimal(Maze_t *maze, Maze_AStarResult_t *result) {
  uint8_t command;

  if (result->optimal_path_index >= result->optimal_path_length) {
    return 0; /* Path complete */
  }

  command = result->optimal_path[result->optimal_path_index].command;
  result->optimal_path_index++;

  /* NOTE: Do NOT update robot_x/y/dir here. Caller handles it. */

  return command;
}
