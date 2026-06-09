import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from topa import TOPAPositioner
from mapConfig import fligh_zone

def check_line_collision(p1, p2, obs_x, obs_y, obs_r):
    x1, y1 = p1
    x2, y2 = p2
    vx = x2 - x1
    vy = y2 - y1
    dr_sq = vx**2 + vy**2
    if dr_sq == 0:
        return np.sqrt((x1 - obs_x)**2 + (y1 - obs_y)**2) < obs_r
    
    t = ((obs_x - x1) * vx + (obs_y - y1) * vy) / dr_sq
    t = max(0.0, min(1.0, t))
    closest_x = x1 + t * vx
    closest_y = y1 + t * vy
    dist = np.sqrt((closest_x - obs_x)**2 + (closest_y - oy)**2) if 'oy' in locals() else np.sqrt((closest_x - obs_x)**2 + (closest_y - obs_y)**2)
    return dist < obs_r

def generate_los_validation_map():
    engine = TOPAPositioner(frequency_hz=5250e6, noise_floor_dbm=-85.0, max_pt_dbm=25.0)
    uav_radius = engine.uav_radius
    
    zone_key = "B_Auditory"
    zone = fligh_zone[zone_key]
    cage_x = float(zone.get("limit_x", 8.0))
    cage_y = float(zone.get("limit_y", 4.0))
    
    obstacles_list = []
    for obs in zone["obstacles"]:
        x, y = obs["position"]
        radius = obs["radius"]
        obstacles_list.append({"x": float(x), "y": float(y), "radius": float(radius)})
    
    uav_1 = (0.5, 2.0)
    uav_2 = (0.5, 2.0)
    uav_3 = (4.0, 2.0)

    uav_pos = [uav_1, uav_2, uav_3]
    
    scenarios = [
        {
            "title": "Scenario 1: EdgeNode in Clear Region",
            "edge_node": (6.0, 5.0)
        },
        {
            "title": "Scenario 2: EdgeNode Obstructed by Pillar 1",
            "edge_node": (11.0, 2.0)
        },
        {
            "title": "Scenario 3: EdgeNode Obstructed by Pillar 2",
            "edge_node": (10.0, 2.0)
        }
    ]
    
    resolution = 0.05
    x_range = np.arange(-6.0, 13.0, resolution)
    y_range = np.arange(-0.5, 5.5, resolution)
    X, Y = np.meshgrid(x_range, y_range)
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 7), sharey=True)
    cmap_custom = ListedColormap(['#ffcccc', '#d4edda'])
    
    for idx, sc in enumerate(scenarios):
        ax = axes[idx]
        ax.set_aspect('equal')
        ax.grid(True, linestyle=':', alpha=0.5, color='gray')
        
        edge_node_pos = sc["edge_node"]
        current_uav = uav_pos[idx]
        los_grid = np.ones_like(X)
        
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                px = X[i, j]
                py = Y[i, j]
                
                if not (0 <= px <= cage_x and 0 <= py <= cage_y):
                    los_grid[i, j] = 1
                    continue
                
                for obs in obstacles_list:
                    if check_line_collision(current_uav, (px, py), obs["x"], obs["y"], obs["radius"]):
                        los_grid[i, j] = 0
                        break

        ax.pcolormesh(X, Y, los_grid, cmap=cmap_custom, shading='nearest', alpha=0.9, zorder=0)
        
        cage_label = 'Flight Cage Limits (8m x 4m)' if idx == 0 else ""

        rect_cage = plt.Rectangle((0, 0), cage_x, cage_y, fill=False, edgecolor='#dc3545', 
                                  linestyle='--', linewidth=2.5, label=cage_label, zorder=1)
        ax.add_patch(rect_cage)
        
        for obs_idx, obs in enumerate(obstacles_list):
            circle_phys = plt.Circle((obs["x"], obs["y"]), obs["radius"], color='#6c757d', alpha=0.9, zorder=4)
            ax.add_patch(circle_phys)
            
            circle_cspace = plt.Circle((obs["x"], obs["y"]), obs["radius"] + uav_radius, 
                                       fill=False, edgecolor='#ffc107', linestyle=':', linewidth=2.0, zorder=3)
            ax.add_patch(circle_cspace)
            
            if idx == 0 and obs_idx == 0:
                circle_phys.set_label('Physical Obstacle Footprint')
                circle_cspace.set_label(f'Inflated C-Space Boundary (+{uav_radius}m)')
        
        circle_drone = plt.Circle((current_uav[0], current_uav[1]), uav_radius, fill=True, color='#28a745', alpha=0.4, zorder=5)
        ax.add_patch(circle_drone)
        ax.plot(current_uav[0], current_uav[1], 'go', markersize=8, markeredgecolor='black', zorder=6)
        
        is_obstructed = False
        for obs in obstacles_list:
            if check_line_collision(current_uav, edge_node_pos, obs["x"], obs["y"], obs["radius"]):
                is_obstructed = True
                break
        
        link_color = '#dc3545' if is_obstructed else '#28a745'
        
        ax.plot([current_uav[0], edge_node_pos[0]], [current_uav[1], edge_node_pos[1]], 
                color=link_color, linestyle='-', linewidth=2.5, zorder=2)
        
        ax.plot(edge_node_pos[0], edge_node_pos[1], 'rs', markersize=12, markeredgecolor='black', zorder=5)
        
        if idx == 0:
            ax.plot([], [], 'go', markersize=8, markeredgecolor='black', label='UAV Position')
            ax.plot([], [], 'rs', markersize=12, markeredgecolor='black', label='EdgeNode Position')
            ax.plot([], [], color='#28a745', linestyle='-', linewidth=2.5, label='Cleared Line-of-Sight')
            ax.plot([], [], color='#dc3545', linestyle='-', linewidth=2.5, label='Obstructed Line-of-Sight')
            
        ax.set_title(sc["title"], fontsize=12, fontweight='bold', pad=10)
        ax.set_xlabel('X-Axis (Meters)', fontsize=10)
        if idx == 0:
            ax.set_ylabel('Y-Axis (Meters)', fontsize=10)
            
        ax.set_xlim(-6.5, 13.5)
        ax.set_ylim(-0.5, cage_y + 1.5)

    plt.suptitle('TOPA Verification: Variable UAV Ray-Casting Map vs Variable EdgeNode Locations', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    fig.legend(loc='lower center', bbox_to_anchor=(0.5, 0.02), ncol=3, fontsize=10, frameon=True, shadow=True)
    plt.tight_layout(rect=[0, 0.08, 1, 0.93])
    
    output_filename = "topa_los_geometric_validation.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"Validation map exported to: {output_filename}")
    plt.show()

if __name__ == "__main__":
    generate_los_validation_map()