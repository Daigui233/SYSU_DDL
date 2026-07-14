"""Bird's-eye road-mask diagnostics and center-biased route experiments."""

import heapq
import time

import cv2
import numpy as np


def build_birds_eye_mask(
        mask, top_y_ratio=0.32, top_left_x_ratio=0.15,
        top_right_x_ratio=0.85, bottom_left_x_ratio=0.02,
        bottom_right_x_ratio=0.98, destination_margin_ratio=0.08):
    """Warp a forward-view binary road mask into a diagnostic bird's-eye view."""
    binary = (np.asarray(mask) != 0).astype(np.uint8)
    if binary.ndim != 2 or not binary.size:
        raise ValueError("road mask must be a non-empty 2D array")
    height, width = binary.shape
    max_x = float(max(0, width - 1))
    max_y = float(max(0, height - 1))

    top_y = float(np.clip(top_y_ratio, 0.0, 0.95)) * max_y
    source = np.asarray([
        [float(np.clip(top_left_x_ratio, 0.0, 1.0)) * max_x, top_y],
        [float(np.clip(top_right_x_ratio, 0.0, 1.0)) * max_x, top_y],
        [float(np.clip(bottom_right_x_ratio, 0.0, 1.0)) * max_x, max_y],
        [float(np.clip(bottom_left_x_ratio, 0.0, 1.0)) * max_x, max_y],
    ], dtype=np.float32)
    margin = float(np.clip(destination_margin_ratio, 0.0, 0.45)) * max_x
    destination = np.asarray([
        [margin, 0.0],
        [max_x - margin, 0.0],
        [max_x - margin, max_y],
        [margin, max_y],
    ], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(source, destination)
    birds_eye = cv2.warpPerspective(
        binary, matrix, (width, height), flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    birds_eye = (birds_eye != 0).astype(np.uint8)
    return birds_eye, matrix, source, destination


def detect_frontier_exits(
        birds_eye_mask, band_width=4, side_reach_ratio=0.70,
        minimum_area=6):
    """Return distinct road intervals touching the far U-shaped frontier."""
    mask = (np.asarray(birds_eye_mask) != 0).astype(np.uint8)
    if mask.ndim != 2 or not mask.size:
        return [], np.zeros(mask.shape, dtype=np.uint8)
    height, width = mask.shape
    band = max(1, min(int(band_width), max(1, min(height, width) // 3)))
    side_end = max(band, min(
        height, int(round(height * float(np.clip(
            side_reach_ratio, 0.05, 1.0))))))
    frontier = np.zeros_like(mask)
    frontier[:band, :] = 1
    frontier[:side_end, :band] = 1
    frontier[:side_end, width - band:] = 1
    samples = mask & frontier

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        samples, connectivity=8)
    exits = []
    min_area = max(1, int(minimum_area))
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        component = labels == label
        top_contact = int(np.count_nonzero(component[:band, :]))
        left_contact = int(np.count_nonzero(component[:side_end, :band]))
        right_contact = int(np.count_nonzero(
            component[:side_end, width - band:]))
        contact_counts = {
            "top": top_contact,
            "left": left_contact,
            "right": right_contact,
        }
        boundary = max(contact_counts, key=contact_counts.get)
        center = centroids[label]
        exits.append({
            "center_xy": (float(center[0]), float(center[1])),
            "area": area,
            "boundary": boundary,
            "contact": int(contact_counts[boundary]),
        })
    exits.sort(key=lambda item: (
        item["center_xy"][0], item["center_xy"][1]))
    return exits, frontier


def road_clearance_map(birds_eye_mask):
    """Distance to the nearest road edge, including the image boundary."""
    mask = (np.asarray(birds_eye_mask) != 0).astype(np.uint8)
    padded = cv2.copyMakeBorder(
        mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
    return cv2.distanceTransform(
        padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]


def find_near_road_start(
        birds_eye_mask, clearance=None, band_width=6,
        lower_side_start_ratio=0.58, search_radius=8):
    """Locate the incoming road and move its boundary contact toward its center."""
    mask = (np.asarray(birds_eye_mask) != 0).astype(np.uint8)
    if mask.ndim != 2 or not np.any(mask):
        return None
    height, width = mask.shape
    band = max(1, min(int(band_width), max(1, min(height, width) // 3)))
    samples = np.zeros_like(mask)
    samples[height - band:, :] = mask[height - band:, :]
    if not np.any(samples):
        side_start = max(0, min(
            height - 1, int(round(height * float(np.clip(
                lower_side_start_ratio, 0.0, 0.95))))))
        samples[side_start:, :band] = mask[side_start:, :band]
        samples[side_start:, width - band:] = mask[
            side_start:, width - band:]
    if not np.any(samples):
        return None

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        samples, connectivity=8)
    anchor = np.asarray([(width - 1) * 0.5, height - 1], dtype=np.float32)
    best_label = None
    best_score = None
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        center = np.asarray(centroids[label], dtype=np.float32)
        score = float(np.linalg.norm(center - anchor)) - min(area, 100) * 0.1
        if best_score is None or score < best_score:
            best_label = label
            best_score = score
    if best_label is None:
        return None

    contact = (labels == best_label).astype(np.uint8)
    radius = max(1, int(search_radius))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1))
    candidates = (cv2.dilate(contact, kernel) != 0) & (mask != 0)
    if clearance is None:
        clearance = road_clearance_map(mask)
    scores = np.where(candidates, clearance, -1.0)
    flat_index = int(np.argmax(scores))
    if float(scores.flat[flat_index]) < 0.0:
        center = centroids[best_label]
        return (int(round(center[0])), int(round(center[1])))
    y, x = np.unravel_index(flat_index, scores.shape)
    return (int(x), int(y))


def _snap_to_mask(point_xy, mask, radius=8):
    x = int(round(point_xy[0]))
    y = int(round(point_xy[1]))
    height, width = mask.shape
    if 0 <= x < width and 0 <= y < height and mask[y, x]:
        return (x, y)
    best = None
    best_distance = None
    for candidate_y in range(max(0, y - radius), min(height, y + radius + 1)):
        for candidate_x in range(max(0, x - radius), min(width, x + radius + 1)):
            if not mask[candidate_y, candidate_x]:
                continue
            distance = (candidate_x - x) ** 2 + (candidate_y - y) ** 2
            if best_distance is None or distance < best_distance:
                best = (candidate_x, candidate_y)
                best_distance = distance
    return best


def plan_center_biased_paths(
        birds_eye_mask, exits, center_weight=6.0, start_band_width=6,
        start_search_radius=8, planning_downsample=2):
    """Build one Dijkstra tree and backtrack one shared-trunk path per exit."""
    mask = (np.asarray(birds_eye_mask) != 0).astype(np.uint8)
    if mask.ndim != 2 or not np.any(mask) or not exits:
        return None, [], np.zeros(mask.shape, dtype=np.float32)
    factor = max(1, int(planning_downsample))
    if factor > 1 and min(mask.shape) >= factor * 8:
        height, width = mask.shape
        small_width = max(1, width // factor)
        small_height = max(1, height // factor)
        reduced = cv2.resize(
            mask.astype(np.float32), (small_width, small_height),
            interpolation=cv2.INTER_AREA)
        reduced = (reduced >= 0.25).astype(np.uint8)
        scale_x = float(width) / float(small_width)
        scale_y = float(height) / float(small_height)
        reduced_exits = []
        for exit_info in exits:
            center = exit_info.get("center_xy", (0.0, 0.0))
            item = dict(exit_info)
            item["center_xy"] = (
                float(center[0]) / scale_x,
                float(center[1]) / scale_y,
            )
            item["original_exit"] = exit_info
            reduced_exits.append(item)
        start, paths, _small_clearance = plan_center_biased_paths(
            reduced, reduced_exits, center_weight=center_weight,
            start_band_width=max(1, int(round(
                float(start_band_width) / scale_y))),
            start_search_radius=max(1, int(round(
                float(start_search_radius) / max(scale_x, scale_y)))),
            planning_downsample=1,
        )
        if start is not None:
            start = (
                int(round(start[0] * scale_x)),
                int(round(start[1] * scale_y)),
            )
        for path_info in paths:
            points = path_info["points_xy"].copy()
            points[:, 0] *= scale_x
            points[:, 1] *= scale_y
            path_info["points_xy"] = points
            goal = path_info["goal_xy"]
            path_info["goal_xy"] = (
                int(round(goal[0] * scale_x)),
                int(round(goal[1] * scale_y)),
            )
            path_info["exit"] = path_info["exit"].get(
                "original_exit", path_info["exit"])
        return start, paths, road_clearance_map(mask)

    clearance = road_clearance_map(mask)
    start = find_near_road_start(
        mask, clearance=clearance, band_width=start_band_width,
        search_radius=start_search_radius)
    if start is None:
        return None, [], clearance

    goals = []
    for exit_info in exits:
        goal = _snap_to_mask(exit_info.get("center_xy", (0, 0)), mask)
        if goal is not None:
            goals.append((goal, exit_info))
    if not goals:
        return start, [], clearance

    height, width = mask.shape
    size = height * width
    start_index = start[1] * width + start[0]
    goal_indices = {point[1] * width + point[0] for point, _info in goals}
    remaining = set(goal_indices)
    distances = np.full(size, np.inf, dtype=np.float32)
    parents = np.full(size, -1, dtype=np.int32)
    visited = np.zeros(size, dtype=np.uint8)
    distances[start_index] = 0.0
    queue = [(0.0, start_index)]
    neighbor_steps = (
        (-1, -1, 1.41421356), (0, -1, 1.0), (1, -1, 1.41421356),
        (-1, 0, 1.0),                         (1, 0, 1.0),
        (-1, 1, 1.41421356),  (0, 1, 1.0),  (1, 1, 1.41421356),
    )
    weight = max(0.0, float(center_weight))
    while queue and remaining:
        distance, index = heapq.heappop(queue)
        if visited[index]:
            continue
        visited[index] = 1
        if index in remaining:
            remaining.remove(index)
            if not remaining:
                break
        y, x = divmod(index, width)
        for dx, dy, step_length in neighbor_steps:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            if not mask[ny, nx]:
                continue
            neighbor = ny * width + nx
            if visited[neighbor]:
                continue
            edge_penalty = 1.0 + weight / (float(clearance[ny, nx]) + 1.0)
            candidate = distance + step_length * edge_penalty
            if candidate >= float(distances[neighbor]):
                continue
            distances[neighbor] = candidate
            parents[neighbor] = index
            heapq.heappush(queue, (candidate, neighbor))

    paths = []
    for goal, exit_info in goals:
        goal_index = goal[1] * width + goal[0]
        if not np.isfinite(distances[goal_index]):
            continue
        indices = []
        index = goal_index
        while index >= 0:
            indices.append(index)
            if index == start_index:
                break
            index = int(parents[index])
        if not indices or indices[-1] != start_index:
            continue
        indices.reverse()
        points = np.asarray([
            (item % width, item // width) for item in indices
        ], dtype=np.float32)
        paths.append({
            "points_xy": points,
            "goal_xy": goal,
            "cost": float(distances[goal_index]),
            "exit": exit_info,
        })
    return start, paths, clearance


def _nearest_binary_point(binary, point_xy):
    ys, xs = np.nonzero(binary)
    if not len(xs):
        return None
    dx = xs.astype(np.float32) - float(point_xy[0])
    dy = ys.astype(np.float32) - float(point_xy[1])
    index = int(np.argmin(dx * dx + dy * dy))
    return (int(xs[index]), int(ys[index]))


def plan_medial_axis_paths(
        birds_eye_mask, exits, planning_downsample=2,
        start_search_radius=8):
    """Trace shared-trunk routes on the medial axis of a filled road mask."""
    mask = (np.asarray(birds_eye_mask) != 0).astype(np.uint8)
    clearance = road_clearance_map(mask)
    factor = max(1, int(planning_downsample))
    if factor > 1 and min(mask.shape) >= factor * 8:
        height, width = mask.shape
        small_width = max(1, width // factor)
        small_height = max(1, height // factor)
        reduced = cv2.resize(
            mask.astype(np.float32), (small_width, small_height),
            interpolation=cv2.INTER_AREA)
        reduced = (reduced >= 0.25).astype(np.uint8)
        scale_x = float(width) / float(small_width)
        scale_y = float(height) / float(small_height)
        reduced_exits = []
        for exit_info in exits:
            center = exit_info.get("center_xy", (0.0, 0.0))
            item = dict(exit_info)
            item["center_xy"] = (
                float(center[0]) / scale_x,
                float(center[1]) / scale_y,
            )
            item["original_exit"] = exit_info
            reduced_exits.append(item)
        start, paths, small_skeleton, _small_clearance = (
            plan_medial_axis_paths(
                reduced, reduced_exits, planning_downsample=1,
                start_search_radius=max(1, int(round(
                    float(start_search_radius) /
                    max(scale_x, scale_y))))))
        if start is not None:
            start = (
                int(round(start[0] * scale_x)),
                int(round(start[1] * scale_y)),
            )
        for path_info in paths:
            points = path_info["points_xy"].copy()
            points[:, 0] *= scale_x
            points[:, 1] *= scale_y
            path_info["points_xy"] = points
            goal = path_info["goal_xy"]
            path_info["goal_xy"] = (
                int(round(goal[0] * scale_x)),
                int(round(goal[1] * scale_y)),
            )
            path_info["exit"] = path_info["exit"].get(
                "original_exit", path_info["exit"])
        skeleton = cv2.resize(
            small_skeleton, (width, height), interpolation=cv2.INTER_NEAREST)
        return start, paths, skeleton, clearance

    ximgproc = getattr(cv2, "ximgproc", None)
    thinning = getattr(ximgproc, "thinning", None)
    if not callable(thinning) or not np.any(mask) or not exits:
        return None, [], np.zeros_like(mask), clearance
    skeleton = (thinning(
        mask * 255,
        thinningType=getattr(
            ximgproc, "THINNING_ZHANGSUEN", 0)) != 0).astype(np.uint8)
    start_hint = find_near_road_start(
        mask, clearance=clearance,
        search_radius=start_search_radius)
    if start_hint is None:
        return None, [], skeleton, clearance
    start = _nearest_binary_point(skeleton, start_hint)
    if start is None:
        return None, [], skeleton, clearance

    goals = []
    for exit_info in exits:
        goal = _nearest_binary_point(
            skeleton, exit_info.get("center_xy", (0.0, 0.0)))
        if goal is not None:
            goals.append((goal, exit_info))
    if not goals:
        return start, [], skeleton, clearance

    height, width = skeleton.shape
    size = height * width
    start_index = start[1] * width + start[0]
    goal_indices = {goal[1] * width + goal[0] for goal, _info in goals}
    remaining = set(goal_indices)
    distances = np.full(size, np.inf, dtype=np.float32)
    parents = np.full(size, -1, dtype=np.int32)
    visited = np.zeros(size, dtype=np.uint8)
    distances[start_index] = 0.0
    queue = [(0.0, start_index)]
    neighbor_steps = (
        (-1, -1, 1.41421356), (0, -1, 1.0), (1, -1, 1.41421356),
        (-1, 0, 1.0),                         (1, 0, 1.0),
        (-1, 1, 1.41421356),  (0, 1, 1.0),  (1, 1, 1.41421356),
    )
    while queue and remaining:
        distance, index = heapq.heappop(queue)
        if visited[index]:
            continue
        visited[index] = 1
        if index in remaining:
            remaining.remove(index)
            if not remaining:
                break
        y, x = divmod(index, width)
        for dx, dy, step_length in neighbor_steps:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue
            if not skeleton[ny, nx]:
                continue
            neighbor = ny * width + nx
            candidate = distance + step_length
            if candidate >= float(distances[neighbor]):
                continue
            distances[neighbor] = candidate
            parents[neighbor] = index
            heapq.heappush(queue, (candidate, neighbor))

    paths = []
    used_goals = set()
    for goal, exit_info in goals:
        goal_index = goal[1] * width + goal[0]
        if goal_index in used_goals or not np.isfinite(distances[goal_index]):
            continue
        used_goals.add(goal_index)
        indices = []
        index = goal_index
        while index >= 0:
            indices.append(index)
            if index == start_index:
                break
            index = int(parents[index])
        if not indices or indices[-1] != start_index:
            continue
        indices.reverse()
        points = np.asarray([
            (item % width, item // width) for item in indices
        ], dtype=np.float32)
        paths.append({
            "points_xy": points,
            "goal_xy": goal,
            "cost": float(distances[goal_index]),
            "exit": exit_info,
            "source": "medial_axis",
        })
    return start, paths, skeleton, clearance


def analyze_road_mask(mask, **parameters):
    """Build BEV, frontier topology, and experimental center-biased paths."""
    perspective_keys = {
        "top_y_ratio", "top_left_x_ratio", "top_right_x_ratio",
        "bottom_left_x_ratio", "bottom_right_x_ratio",
        "destination_margin_ratio",
    }
    perspective = {
        key: value for key, value in parameters.items()
        if key in perspective_keys
    }
    frontier = {
        key: value for key, value in parameters.items()
        if key in {"band_width", "side_reach_ratio", "minimum_area"}
    }
    planning = {
        key: value for key, value in parameters.items()
        if key in {
            "center_weight", "start_band_width", "start_search_radius",
            "planning_downsample"}
    }
    started = time.perf_counter()
    birds_eye, matrix, source, destination = build_birds_eye_mask(
        mask, **perspective)
    exits, frontier_mask = detect_frontier_exits(
        birds_eye, **frontier)
    start, paths, skeleton, clearance = plan_medial_axis_paths(
        birds_eye, exits,
        planning_downsample=planning.get("planning_downsample", 2),
        start_search_radius=planning.get("start_search_radius", 8))
    planner = "medial_axis"
    if len(paths) != len(exits):
        start, paths, clearance = plan_center_biased_paths(
            birds_eye, exits, **planning)
        skeleton = np.zeros_like(birds_eye)
        planner = "cost_dijkstra_fallback"
    inverse_matrix = np.linalg.inv(matrix)
    return {
        "mask": birds_eye,
        "matrix": matrix,
        "inverse_matrix": inverse_matrix,
        "source_points": source,
        "destination_points": destination,
        "frontier_mask": frontier_mask,
        "exits": exits,
        "exit_count": len(exits),
        "start_xy": start,
        "paths": paths,
        "skeleton": skeleton,
        "planner": planner,
        "clearance": clearance,
        "planning_ms": (time.perf_counter() - started) * 1000.0,
    }
