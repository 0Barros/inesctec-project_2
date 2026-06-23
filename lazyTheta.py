import numpy as np
import heapq

class LazyThetaStarPlanner:
    def __init__(self, cage_x, cage_y, obstacles, uav_radius, resolution=0.1):
        self.cage_x = cage_x
        self.cage_y = cage_y
        self.resolution = resolution
        self.uav_radius = uav_radius
        self.obstacles = obstacles
        
        self.width = int(np.ceil(cage_x / resolution)) + 1
        self.height = int(np.ceil(cage_y / resolution)) + 1

    def _grid_to_world(self, grid_pos):
        return (grid_pos[0] * self.resolution, grid_pos[1] * self.resolution)

    def _world_to_grid(self, world_pos):
        gx = int(max(0, min(self.width - 1, round(world_pos[0] / self.resolution))))
        gy = int(max(0, min(self.height - 1, round(world_pos[1] / self.resolution))))
        return (gx, gy)

    def _is_valid_node(self, grid_pos):
        x, y = self._grid_to_world(grid_pos)
        
        if x < self.uav_radius or x > (self.cage_x - self.uav_radius):
            return False
        if y < self.uav_radius or y > (self.cage_y - self.uav_radius):
            return False
            
        for obs in self.obstacles:
            dist = np.sqrt((x - obs["x"])**2 + (y - obs["y"])**2)
            if dist < (obs["radius"] + self.uav_radius):
                return False
        return True

    def _line_of_sight(self, p1, p2):
        w1 = self._grid_to_world(p1)
        w2 = self._grid_to_world(p2)
        
        vx = w2[0] - w1[0]
        vy = w2[1] - w1[1]
        dist_total = np.sqrt(vx**2 + vy**2)
        if dist_total == 0:
            return True
            
        steps = int(np.ceil(dist_total / (self.resolution / 2.0)))
        for s in range(steps + 1):
            t = s / steps
            cx = w1[0] + t * vx
            cy = w1[1] + t * vy
            
            if cx < self.uav_radius or cx > (self.cage_x - self.uav_radius):
                return False
            if cy < self.uav_radius or cy > (self.cage_y - self.uav_radius):
                return False
                
            for obs in self.obstacles:
                if np.sqrt((cx - obs["x"])**2 + (cy - obs["y"])**2) < (obs["radius"] + self.uav_radius):
                    return False
        return True

    def _get_neighbors(self, pos):
        neighbors = []
        moves = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        for dx, dy in moves:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self._is_valid_node((nx, ny)):
                    neighbors.append((nx, ny))
        return neighbors

    def _heuristic(self, p1, p2):
        w1 = self._grid_to_world(p1)
        w2 = self._grid_to_world(p2)
        return np.sqrt((w1[0] - w2[0])**2 + (w1[1] - w2[1])**2)

    def plan_path(self, start_world, goal_world):
        start_grid = self._world_to_grid(start_world)
        goal_grid = self._world_to_grid(goal_world)
        
        if not self._is_valid_node(start_grid):
            start_grid = self._find_closest_valid(start_grid)
        if not self._is_valid_node(goal_grid):
            goal_grid = self._find_closest_valid(goal_grid)

        open_list = []
        heapq.heappush(open_list, (0 + self._heuristic(start_grid, goal_grid), start_grid))
        
        g_score = {start_grid: 0.0}
        parent = {start_grid: start_grid}
        closed_set = set()

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal_grid:
                return self._reconstruct_path(parent, current)

            if current in closed_set:
                continue
                
            closed_set.add(current)

            p_curr = parent[current]
            if p_curr != current and not self._line_of_sight(p_curr, current):
                neighbors_p = self._get_neighbors(current)
                best_g = float('inf')
                best_parent = current
                for n in neighbors_p:
                    if n in closed_set:
                        cost = g_score[n] + self._heuristic(n, current)
                        if cost < best_g:
                            best_g = cost
                            best_parent = n
                g_score[current] = best_g
                parent[current] = best_parent

            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set:
                    continue

                p_curr = parent[current]
                potential_g = g_score[p_curr] + self._heuristic(p_curr, neighbor)

                if neighbor not in g_score or potential_g < g_score[neighbor]:
                    g_score[neighbor] = potential_g
                    parent[neighbor] = p_curr
                    f_score = potential_g + self._heuristic(neighbor, goal_grid)
                    heapq.heappush(open_list, (f_score, neighbor))

        return [start_world, goal_world]

    def _find_closest_valid(self, node):
        queue = [node]
        visited = {node}
        while queue:
            curr = queue.pop(0)
            if self._is_valid_node(curr):
                return curr
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nxt = (curr[0] + dx, curr[1] + dy)
                if 0 <= nxt[0] < self.width and 0 <= nxt[1] < self.height and nxt not in visited:
                    visited.add(nxt)
                    queue.append(nxt)
        return node

    def _reconstruct_path(self, parent, current):
        path = []
        while parent[current] != current:
            path.append(self._grid_to_world(current))
            current = parent[current]
        path.append(self._grid_to_world(current))
        path.reverse()
        return path