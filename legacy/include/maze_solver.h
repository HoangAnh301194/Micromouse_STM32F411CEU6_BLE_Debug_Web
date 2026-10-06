#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include <stdint.h>

#define MAZE_SIZE 5
#define MAZE_CENTER_X1 4
#define MAZE_CENTER_Y1 4
#define MAZE_CENTER_X2 4
#define MAZE_CENTER_Y2 4

/* Helper: check if (x,y) is inside the center region */
#define IS_CENTER_CELL(x, y)                                                   \
  ((x) >= MAZE_CENTER_X1 && (x) <= MAZE_CENTER_X2 && (y) >= MAZE_CENTER_Y1 &&  \
   (y) <= MAZE_CENTER_Y2)

#define VISITED_CELL_PENALTY 50

/* Wall update mode: 0=add-only (never erase known walls), 1=full update (can
 * erase) */
#ifndef MAZE_WALL_ADD_ONLY
#define MAZE_WALL_ADD_ONLY 0
#endif

/* Wall direction bits (for walls[][] bitfield) */
#define WALL_NORTH 0x01
#define WALL_EAST 0x02
#define WALL_SOUTH 0x04
#define WALL_WEST 0x08

/* Confidence bits: which directions have been OBSERVED from this cell */
#define CONF_NORTH 0x01
#define CONF_EAST 0x02
#define CONF_SOUTH 0x04
#define CONF_WEST 0x08

/* Magic value to mark persistent map as valid */
#define MAZE_PERSISTENT_MAGIC 0xA5

/* ========================================================================== */
/* PERSISTENT MAP — survives across runs, NEVER auto-reset                    */
/* ========================================================================== */
typedef struct {
  uint8_t walls[MAZE_SIZE][MAZE_SIZE];      /* Accumulated wall knowledge */
  uint8_t confidence[MAZE_SIZE][MAZE_SIZE]; /* Bitfield: observed directions */
  uint8_t visit_count[MAZE_SIZE]
                     [MAZE_SIZE]; /* Times robot passed through cell */

  uint8_t total_runs;           /* How many explore runs completed */
  uint8_t center_reached_count; /* How many times center was reached */
  uint8_t is_valid;             /* MAZE_PERSISTENT_MAGIC = data valid */
  uint8_t fully_explored;       /* 1 = all reachable cells observed */
} Maze_Persistent_t;

/* ========================================================================== */
/* PER-RUN WORKING DATA — reset every run                                     */
/* ========================================================================== */
typedef struct {
  uint8_t walls[MAZE_SIZE][MAZE_SIZE];  /* Copy from persistent + live update */
  uint16_t flood[MAZE_SIZE][MAZE_SIZE]; /* Flood fill (computed at runtime) */
  uint8_t visited[MAZE_SIZE][MAZE_SIZE]; /* Visited in THIS run only */

  uint8_t robot_x;
  uint8_t robot_y;
  uint8_t robot_dir;

  uint8_t path[256];
  uint8_t path_len;

  /* Reference to persistent map — if non-NULL, Maze_UpdateWalls() will
   * NOT modify wall directions that were already observed (confidence != 0)
   * in the persistent map.  Set by Maze_Persistent_LoadInto(). */
  Maze_Persistent_t *persistent_ref;
} Maze_t;

/* ========================================================================== */
/* A* PATH TYPES                                                              */
/* ========================================================================== */
typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t direction;
  uint16_t g_cost;
  uint16_t f_cost;
  uint8_t command;
} AStarNode_t;

typedef struct {
  AStarNode_t optimal_path[256];
  uint8_t optimal_path_length;
  uint8_t optimal_path_index;
} Maze_AStarResult_t;

/* ========================================================================== */
/* PERSISTENT MAP API                                                         */
/* ========================================================================== */

/** Initialize persistent map (call once at startup) */
void Maze_Persistent_Init(Maze_Persistent_t *pm);

/** Reset persistent map (erase all knowledge) */
void Maze_Persistent_Reset(Maze_Persistent_t *pm);

/** Copy persistent walls into per-run maze (call at start of each explore) */
void Maze_Persistent_LoadInto(Maze_Persistent_t *pm, Maze_t *maze);

/** Sync newly discovered walls from per-run maze back to persistent */
void Maze_Persistent_SyncFrom(Maze_Persistent_t *pm, Maze_t *maze);

/** Update confidence for a cell (call after wall observation) */
void Maze_Persistent_MarkObserved(Maze_Persistent_t *pm, uint8_t x, uint8_t y,
                                  uint8_t robot_dir);

/** Increment visit count for a cell */
void Maze_Persistent_IncrementVisit(Maze_Persistent_t *pm, uint8_t x,
                                    uint8_t y);

/** Get exploration coverage percentage (0-100) */
uint8_t Maze_Persistent_GetCoverage(Maze_Persistent_t *pm);

/** Check if persistent map has enough data for A* */
uint8_t Maze_Persistent_IsReadyForAStar(Maze_Persistent_t *pm);

/** Print persistent map status via UART */
void Maze_Persistent_PrintStatus(Maze_Persistent_t *pm);

/* ========================================================================== */
/* PER-RUN MAZE API                                                           */
/* ========================================================================== */

void Maze_Init(Maze_t *maze);
void Maze_UpdateWalls(Maze_t *maze, uint8_t left, uint8_t front, uint8_t right);
void Maze_AddWalls(Maze_t *maze, uint8_t left, uint8_t front, uint8_t right);
void Maze_FloodFill(Maze_t *maze, uint8_t target_x, uint8_t target_y);
void Maze_FloodFillCenter(Maze_t *maze);
uint8_t Maze_GetNextMove(Maze_t *maze);

/** Smart exploration: uses persistent visit_count to prefer unvisited cells */
uint8_t Maze_GetNextMove_Smart(Maze_t *maze, Maze_Persistent_t *pm);

void Maze_OptimizePath(Maze_t *maze);
uint8_t Maze_IsAtTarget(Maze_t *maze, uint8_t target_x, uint8_t target_y);
uint8_t Maze_IsAtCenter(Maze_t *maze);
void Maze_ResetPosition(Maze_t *maze);
uint16_t Maze_GetDistanceToTarget(Maze_t *maze);
void Maze_Print(Maze_t *maze);
void Maze_PrintCompact(Maze_t *maze);   /* BLE compact hex dump    */
void Maze_SendCellUpdate(Maze_t *maze); /* BLE single-cell update  */

/* ========================================================================== */
/* A* OPTIMAL PATH API                                                        */
/* ========================================================================== */

/** Calculate A* optimal path to center region */
uint8_t Maze_CalculateOptimalPath(Maze_t *maze, Maze_AStarResult_t *result);

/** Get next move from pre-computed A* path (does NOT update robot pos) */
uint8_t Maze_GetNextMove_Optimal(Maze_t *maze, Maze_AStarResult_t *result);

#endif
