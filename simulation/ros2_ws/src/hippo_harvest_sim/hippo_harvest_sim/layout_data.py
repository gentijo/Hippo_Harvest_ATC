GRID_RESOLUTION_M = 0.025
GRID_WIDTH_CELLS = 180
GRID_HEIGHT_CELLS = 135

TABLE_WIDTH_CELLS = 10
TABLE_HEIGHT_CELLS = 5
TABLE_COLUMNS = 4
TABLE_ROWS = 3
TABLE_GAP_CELLS = 20
BUFFER_CELLS = 40
FIRST_TABLE_NW_X = 40
FIRST_TABLE_NW_Y_FROM_NORTH = 40

TABLE_BUFFER_CELLS = 3
WALL_BUFFER_CELLS = 2
WORKSPACE_APPROACH_OFFSET_CELLS = 12
ROBOT_HOME_X_CELLS = 12
ROBOT_STAGING_X_CELLS = 24
ROBOT_RETURN_TRANSIT_X_CELLS = 30
def generate_tables():
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
    for table in generate_tables():
        if table["id"] == workspace_id:
            return table
    raise KeyError(f"Unknown workspace id: {workspace_id}")


def is_in_bounds(cell):
    x, y = cell
    return 0 <= x < GRID_WIDTH_CELLS and 0 <= y < GRID_HEIGHT_CELLS


def hard_occupied_cells():
    occupied = set()
    for table in generate_tables():
        for x in range(table["x_min"], table["x_max"]):
            for y in range(table["y_min"], table["y_max"]):
                occupied.add((x, y))
    return occupied


def planning_occupied_cells():
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
    return hard_occupied_cells()


def free_cells_with_margin(margin=0):
    occupied = planning_occupied_cells()
    free = []
    for x in range(margin, GRID_WIDTH_CELLS - margin):
        for y in range(margin, GRID_HEIGHT_CELLS - margin):
            if (x, y) not in occupied:
                free.append((x, y))
    return free


def default_start_cell():
    return (WALL_BUFFER_CELLS, WALL_BUFFER_CELLS)


def robot_home_cells(robot_count: int = 10):
    if robot_count <= 0:
        return []

    home_x = ROBOT_HOME_X_CELLS
    min_y = WALL_BUFFER_CELLS
    max_y = GRID_HEIGHT_CELLS - WALL_BUFFER_CELLS - 1

    if robot_count == 1:
        candidate_cells = [(home_x, (min_y + max_y) // 2)]
    else:
        span = max_y - min_y
        candidate_cells = []
        for index in range(robot_count):
            ratio = index / (robot_count - 1)
            y = int(round(min_y + ratio * span))
            candidate_cells.append((home_x, y))

    occupied = planning_occupied_cells()
    validated_cells = []
    used_cells = set()
    for x, y in candidate_cells:
        while (x, y) in occupied or (x, y) in used_cells:
            y += 1
            if y > max_y:
                raise RuntimeError("Unable to place all robot home cells along the west wall")
        used_cells.add((x, y))
        validated_cells.append((x, y))

    return validated_cells


def robot_home_cell(robot_index: int, robot_count: int = 10):
    if robot_index < 1 or robot_index > robot_count:
        raise ValueError(f"robot_index must be in [1, {robot_count}], got {robot_index}")
    return robot_home_cells(robot_count)[robot_index - 1]


def robot_staging_cell(robot_index: int, robot_count: int = 10):
    home_x, home_y = robot_home_cell(robot_index, robot_count)
    cell = (ROBOT_STAGING_X_CELLS, home_y)
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free staging cell found for robot{robot_index}")


def robot_return_transit_cell(workspace_id: str):
    _, aisle_y = robot_return_aisle_entry_cell(workspace_id)
    cell = (ROBOT_RETURN_TRANSIT_X_CELLS, aisle_y)
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free return transit cell found for workspace {workspace_id}")


def robot_return_aisle_entry_cell(workspace_id: str):
    workspace_x, workspace_y = workspace_approach_cell(workspace_id)
    occupied = planning_occupied_cells()
    for offset in range(1, workspace_y - WALL_BUFFER_CELLS + 1):
        cell = (workspace_x, workspace_y - offset)
        if is_in_bounds(cell) and cell not in occupied:
            return cell
    raise RuntimeError(f"No free return aisle entry cell found for workspace {workspace_id}")


def robot_return_stage_transit_cell(robot_index: int, robot_count: int = 10):
    _, home_y = robot_home_cell(robot_index, robot_count)
    cell = (ROBOT_RETURN_TRANSIT_X_CELLS, home_y)
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free return stage transit cell found for robot{robot_index}")


def workspace_approach_cell(workspace_id: str):
    table = workspace_by_id(workspace_id)
    cell = table["waypoint_cell"]
    if is_in_bounds(cell) and cell not in planning_occupied_cells():
        return cell
    raise RuntimeError(f"No free south-edge waypoint cell found for workspace {workspace_id}")


def cell_to_pose(cell):
    x, y = cell
    return ((x + 0.5) * GRID_RESOLUTION_M, (y + 0.5) * GRID_RESOLUTION_M)
