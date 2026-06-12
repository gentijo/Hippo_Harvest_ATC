import math

# Shared layout constants used by the map publisher, planners, orchestrator,
# marker publishers, and synthetic localization bounds.
GRID_RESOLUTION_M = 0.025
GRID_WIDTH_CELLS = 180
GRID_HEIGHT_CELLS = 135

# Workspace/table geometry in grid cells.
TABLE_WIDTH_CELLS = 10
TABLE_HEIGHT_CELLS = 5
TABLE_COLUMNS = 4
TABLE_ROWS = 3
TABLE_GAP_CELLS = 20
BUFFER_CELLS = 40
FIRST_TABLE_NW_X = 40
FIRST_TABLE_NW_Y_FROM_NORTH = 40

# Planning margins and robot staging/return lanes in grid cells.
TABLE_BUFFER_CELLS = 3
WALL_BUFFER_CELLS = 2
WORKSPACE_APPROACH_OFFSET_CELLS = 12
ROBOT_HOME_X_CELLS = 12
ROBOT_STAGING_X_CELLS = 24
ROBOT_RETURN_TRANSIT_X_CELLS = 30
ROBOT_EAST_HOME_X_CELLS = GRID_WIDTH_CELLS - ROBOT_HOME_X_CELLS - 1
ROBOT_EAST_STAGING_X_CELLS = GRID_WIDTH_CELLS - ROBOT_STAGING_X_CELLS - 1
ROBOT_EAST_RETURN_TRANSIT_X_CELLS = GRID_WIDTH_CELLS - ROBOT_RETURN_TRANSIT_X_CELLS - 1


def generate_tables():
    # Build the table/workspace descriptions from the constants above. Each table
    # gets an id such as ws1 plus occupied bounds and a south-side approach cell.
    tables = []
    for row in range(TABLE_ROWS):
        top_y = GRID_HEIGHT_CELLS - FIRST_TABLE_NW_Y_FROM_NORTH - row * (
            TABLE_HEIGHT_CELLS + TABLE_GAP_CELLS
        )
        bottom_y = top_y - TABLE_HEIGHT_CELLS
        for col in range(TABLE_COLUMNS):
            left_x = FIRST_TABLE_NW_X + col * (TABLE_WIDTH_CELLS + TABLE_GAP_CELLS)
            right_x = left_x + TABLE_WIDTH_CELLS
            index = row * TABLE_COLUMNS + col + 1
            center_x = left_x + TABLE_WIDTH_CELLS / 2.0
            center_y = bottom_y + TABLE_HEIGHT_CELLS / 2.0
            south_edge_cell = (int(center_x), bottom_y - WORKSPACE_APPROACH_OFFSET_CELLS)
            tables.append(
                {
                    "id": f"ws{index}",
                    "row": row,
                    "col": col,
                    "x_min": left_x,
                    "x_max": right_x,
                    "y_min": bottom_y,
                    "y_max": top_y,
                    "center_x": center_x,
                    "center_y": center_y,
                    "waypoint_cell": south_edge_cell,
                }
            )
    return tables


def workspace_by_id(workspace_id: str):
    # Resolve a workspace id like "ws2" to its generated table record.
    for table in generate_tables():
        if table["id"] == workspace_id:
            return table
    raise KeyError(f"Unknown workspace id: {workspace_id}")


def is_in_bounds(cell):
    # True when a grid cell lies inside the map extents.
    x, y = cell
    return 0 <= x < GRID_WIDTH_CELLS and 0 <= y < GRID_HEIGHT_CELLS


def hard_occupied_cells():
    # Hard obstacles are the actual table footprints.
    occupied = set()
    for table in generate_tables():
        for x in range(table["x_min"], table["x_max"]):
            for y in range(table["y_min"], table["y_max"]):
                occupied.add((x, y))
    return occupied


def planning_occupied_cells():
    # Planning obstacles include hard obstacles plus buffers around tables and walls.
    occupied = set(hard_occupied_cells())

    for table in generate_tables():
        x_min = max(0, table["x_min"] - TABLE_BUFFER_CELLS)
        x_max = min(GRID_WIDTH_CELLS, table["x_max"] + TABLE_BUFFER_CELLS)
        y_min = max(0, table["y_min"] - TABLE_BUFFER_CELLS)
        y_max = min(GRID_HEIGHT_CELLS, table["y_max"] + TABLE_BUFFER_CELLS)
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                occupied.add((x, y))

    for x in range(GRID_WIDTH_CELLS):
        for y in range(WALL_BUFFER_CELLS):
            occupied.add((x, y))
        for y in range(GRID_HEIGHT_CELLS - WALL_BUFFER_CELLS, GRID_HEIGHT_CELLS):
            occupied.add((x, y))

    for y in range(GRID_HEIGHT_CELLS):
        for x in range(WALL_BUFFER_CELLS):
            occupied.add((x, y))
        for x in range(GRID_WIDTH_CELLS - WALL_BUFFER_CELLS, GRID_WIDTH_CELLS):
            occupied.add((x, y))

    return occupied


def occupied_cells():
    # Compatibility wrapper for callers that want only hard occupied cells.
    return hard_occupied_cells()


def free_cells_with_margin(margin=0):
    # Returns free cells after applying planning obstacles and an optional border margin.
    occupied = planning_occupied_cells()
    free = []
    for x in range(margin, GRID_WIDTH_CELLS - margin):
        for y in range(margin, GRID_HEIGHT_CELLS - margin):
            if (x, y) not in occupied:
                free.append((x, y))
    return free


def default_start_cell():
    # Single-robot fallback start near the southwest corner of the map.
    return (WALL_BUFFER_CELLS, WALL_BUFFER_CELLS)


def robot_home_cells(robot_count: int = 10):
    # Bias the fleet toward the west side so the demo naturally produces more
    # crossing traffic through the shared aisles while still keeping home poses
    # separated enough that robots do not spawn on top of each other.
    if robot_count <= 0:
        return []

    min_y = WALL_BUFFER_CELLS
    max_y = GRID_HEIGHT_CELLS - WALL_BUFFER_CELLS - 1
    west_count = min(robot_count, max(1, math.ceil(robot_count * 0.8)))
    east_count = max(0, robot_count - west_count)

    def side_candidates(home_x: int, count: int) -> list[tuple[int, int]]:
        if count <= 0:
            return []
        if count == 1:
            return [(home_x, (min_y + max_y) // 2)]
        span = max_y - min_y
        cells = []
        for index in range(count):
            ratio = index / (count - 1)
            y = int(round(min_y + ratio * span))
            cells.append((home_x, y))
        return cells

    candidate_cells = [
        *side_candidates(ROBOT_HOME_X_CELLS, west_count),
        *side_candidates(ROBOT_EAST_HOME_X_CELLS, east_count),
    ]

    occupied = planning_occupied_cells()
    validated_cells = []
    used_cells = set()
    for x, y in candidate_cells:
        while (x, y) in occupied or (x, y) in used_cells:
            y += 1
            if y > max_y:
                raise RuntimeError("Unable to place all robot home cells along side walls")
        used_cells.add((x, y))
        validated_cells.append((x, y))

    return validated_cells


def robot_home_cell(robot_index: int, robot_count: int = 10):
    # 1-based robot index helper for launch files and per-robot nodes.
    if robot_index < 1 or robot_index > robot_count:
        raise ValueError(f"robot_index must be in [1, {robot_count}], got {robot_index}")
    return robot_home_cells(robot_count)[robot_index - 1]


def robot_home_yaw(robot_index: int, robot_count: int = 10):
    # West-side robots face east; east-side robots face west toward the aisle.
    home_x, _ = robot_home_cell(robot_index, robot_count)
    return 0.0 if home_x < GRID_WIDTH_CELLS // 2 else 3.141592653589793


def robot_staging_cell(robot_index: int, robot_count: int = 10):
    # Staging cells sit inward from each robot's home row and are used before tasks.
    home_x, home_y = robot_home_cell(robot_index, robot_count)
    staging_x = ROBOT_STAGING_X_CELLS if home_x < GRID_WIDTH_CELLS // 2 else ROBOT_EAST_STAGING_X_CELLS
    cell = (staging_x, home_y)
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free staging cell found for robot{robot_index}")


def robot_return_transit_cell(workspace_id: str):
    # Return transit cells move robots back into the west-side return lane.
    _, aisle_y = robot_return_aisle_entry_cell(workspace_id)
    cell = (ROBOT_RETURN_TRANSIT_X_CELLS, aisle_y)
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free return transit cell found for workspace {workspace_id}")


def robot_return_aisle_entry_cell(workspace_id: str):
    # Find the first free cell south of a workspace approach point for exiting.
    workspace_x, workspace_y = workspace_approach_cell(workspace_id)
    occupied = planning_occupied_cells()
    for offset in range(1, workspace_y - WALL_BUFFER_CELLS + 1):
        cell = (workspace_x, workspace_y - offset)
        if is_in_bounds(cell) and cell not in occupied:
            return cell
    raise RuntimeError(f"No free return aisle entry cell found for workspace {workspace_id}")


def robot_return_stage_transit_cell(robot_index: int, robot_count: int = 10):
    # Transit point on the robot's home row before returning to staging/home.
    home_x, home_y = robot_home_cell(robot_index, robot_count)
    transit_x = (
        ROBOT_RETURN_TRANSIT_X_CELLS
        if home_x < GRID_WIDTH_CELLS // 2
        else ROBOT_EAST_RETURN_TRANSIT_X_CELLS
    )
    cell = (transit_x, home_y)
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free return stage transit cell found for robot{robot_index}")


def workspace_approach_cell(workspace_id: str):
    # Main task waypoint for a workspace: the free south-side approach cell.
    table = workspace_by_id(workspace_id)
    cell = table["waypoint_cell"]
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free south-edge waypoint cell found for workspace {workspace_id}")


def cell_to_pose(cell):
    # Convert grid-cell coordinates to the center of that cell in map-frame meters.
    x, y = cell
    return ((x + 0.5) * GRID_RESOLUTION_M, (y + 0.5) * GRID_RESOLUTION_M)
