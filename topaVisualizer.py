import numpy as np
import matplotlib.pyplot as plt
import time
from topa import TOPAPositioner
from mapConfig import fligh_zone  
from lazyTheta import LazyThetaStarPlanner

def create_moving_target_simulation():
    engine = TOPAPositioner(frequency_hz=5250e6, noise_floor_dbm=-85.0, max_pt_dbm=25.0)
    
    zone_key = "B_Auditory"
    zone = fligh_zone[zone_key]
    
    cage_config = {
        "limit_x": float(zone.get("limit_x", 8.0)),   
        "limit_y": float(zone.get("limit_y", 4.0)),   
        "limit_z": 3.0  
    }
    
    obstacles_list = []
    obstacles_input = []
    for obs in zone["obstacles"]:
        x, y = obs["position"]
        radius = obs["radius"]  
        obstacles_list.append({
            "x": float(x),
            "y": float(y),
            "radius": float(radius),
            "height": 4.0  
        })
        obstacles_input.append({
            "x": float(x),
            "y": float(y),
            "radius": float(radius)
        })
    
    uav_current_pos = [0.8, 2.0, 1.5]
    uav_speed_step = 0.35 
    
    num_frames = 50
    edge_node_y_trajectory = np.linspace(0.5, 3.5, num_frames)
    edge_node_x = 11.0  
    constant_snr_demand = 16.0 
    
    # Inicialização do Planeador Lazy Theta* fora do ciclo (Garante eficiência)
    planner = LazyThetaStarPlanner(
        cage_x=cage_config["limit_x"], 
        cage_y=cage_config["limit_y"], 
        obstacles=obstacles_input, 
        uav_radius=engine.uav_radius, 
        resolution=0.1
    )
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(11, 6))
    
    print("=" * 80)
    print(f"RUNNING LAZY THETA* OBSTACLE AVOIDANCE SIMULATION - {zone['name']}")
    print("=" * 80)
    
    for frame_idx in range(num_frames):
        ax.clear()
        
        current_target_y = edge_node_y_trajectory[frame_idx]
        edge_node_pos = (edge_node_x, current_target_y, 1.0)
        edge_node_with_snr = (edge_node_pos[0], edge_node_pos[1], edge_node_pos[2], constant_snr_demand)
        
        optimal_pos = engine.compute_topa_optimal_position(
            uav_current_pos, edge_node_with_snr, obstacles_list, cage_config)
        
        # --- ALTERAÇÃO: Planeamento de Caminho com Lazy Theta* ---
        # Calcula os pontos de desvio geométrico entre a posição atual e a meta do TOPA
        waypoints = planner.plan_path((uav_current_pos[0], uav_current_pos[1]), (optimal_pos[0], optimal_pos[1]))
        
        # --- KINEMATICS CONSUMING LAZY THETA* WAYPOINTS ---
        # Se o Lazy Theta* gerou desvios, o drone deve seguir para o próximo waypoint intermédio
        if len(waypoints) > 1:
            next_target = waypoints[1] # O índice 0 é a posição atual, o índice 1 é o próximo passo seguro
            dx = next_target[0] - uav_current_pos[0]
            dy = next_target[1] - uav_current_pos[1]
            dist_to_next = np.sqrt(dx**2 + dy**2)
            
            if dist_to_next > uav_speed_step:
                uav_current_pos[0] += (dx / dist_to_next) * uav_speed_step
                uav_current_pos[1] += (dy / dist_to_next) * uav_speed_step
                uav_current_pos[2] = optimal_pos[2]
            else:
                uav_current_pos[0] = next_target[0]
                uav_current_pos[1] = next_target[1]
                uav_current_pos[2] = optimal_pos[2]
        else:
            uav_current_pos = list(optimal_pos)
            
        opt_x, opt_y, opt_z = uav_current_pos
        
        # --- Real-Time Line of Sight (LoS) Check ---
        los_is_blocked = False
        uav_to_target_dist = np.sqrt((opt_x - edge_node_pos[0])**2 + (opt_y - edge_node_pos[1])**2)
        
        for obs in obstacles_list:
            ox, oy, r = obs["x"], obs["y"], obs["radius"]
            vx = opt_x - edge_node_pos[0]
            vy = opt_y - edge_node_pos[1]
            dr_sq = vx**2 + vy**2
            
            if dr_sq > 0:
                t = ((obs["x"] - edge_node_pos[0]) * vx + (obs["y"] - edge_node_pos[1]) * vy) / dr_sq
                t = max(0.0, min(1.0, t))
                closest_x = edge_node_pos[0] + t * vx
                closest_y = edge_node_pos[1] + t * vy
                segment_dist_to_obs = np.sqrt((closest_x - ox)**2 + (closest_y - oy)**2)
                
                if segment_dist_to_obs < r:
                    los_is_blocked = True
                    break

        los_color = 'red' if los_is_blocked else 'lime'
        los_label = 'Line-of-Sight BLOCKED' if los_is_blocked else 'Line-of-Sight CLEARED'
        
        min_dist_to_obstacle = float('inf')
        for obs in obstacles_list:
            dist = np.sqrt((opt_x - obs["x"])**2 + (opt_y - obs["y"])**2) - obs["radius"]
            min_dist_to_obstacle = min(min_dist_to_obstacle, dist)
        cspace_margin = min_dist_to_obstacle - engine.uav_radius
        
        # --- RENDER ENGINE ---
        ax.set_xlim(-1, edge_node_x + 2)
        ax.set_ylim(-1, cage_config["limit_y"] + 1.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        
        rect = plt.Rectangle((0, 0), cage_config["limit_x"], cage_config["limit_y"], 
                             fill=False, edgecolor='red', linestyle='--', linewidth=2.5, 
                             label=f"Flight Cage Boundary")
        ax.add_patch(rect)
        
        for i, obs in enumerate(obstacles_list):
            ax.add_patch(plt.Circle((obs["x"], obs["y"]), obs["radius"], color='gray', alpha=0.8, zorder=3))
            cspace_r = obs["radius"] + engine.uav_radius
            cspace_circle = plt.Circle((obs["x"], obs["y"]), cspace_r, fill=False, 
                                       edgecolor='orange', linestyle=':', linewidth=2, zorder=2)
            ax.add_patch(cspace_circle)
            if i == 0:
                cspace_circle.set_label('C-Space Danger Zone (0.65m Buffer)')
        
        # Desenha a linha pontilhada do plano de rota completo gerado pelo Lazy Theta* para este frame
        if len(waypoints) > 1:
            w_xs, w_ys = zip(*waypoints)
            ax.plot(w_xs, w_ys, color='blue', linestyle=':', linewidth=1.5, label='Lazy Theta* Planned Path', zorder=2)

        ax.plot(edge_node_pos[0], edge_node_pos[1], 'rs', markersize=12, label='EdgeNode (Moving Target)', zorder=5)
        ax.plot(optimal_pos[0], optimal_pos[1], 'bx', markersize=8, label='TOPA Target Goal', zorder=5)
        ax.plot(opt_x, opt_y, 'go', markersize=10, label='UAV Current Center', zorder=6)
        ax.add_patch(plt.Circle((opt_x, opt_y), engine.uav_radius, fill=True, color='green', alpha=0.3, label='UAV 0.65m Hull Space', zorder=4))
        ax.plot([opt_x, edge_node_pos[0]], [opt_y, edge_node_pos[1]], color=los_color, linewidth=2.5, zorder=1, label=los_label)
        
        ax.set_title(f'TOPA Dynamic Tracker + Lazy Theta*: {zone["name"]} (Frame {frame_idx+1}/{num_frames})', fontsize=11, fontweight='bold')
        info_text = (f"Target Position: ({edge_node_pos[0]:.2f}, {edge_node_pos[1]:.2f})\n"
                    f"Drone Position:  ({opt_x:.2f}, {opt_y:.2f})\n"
                    f"C-Space Clearance:  {cspace_margin:.2f} m\n"
                    f"Link State: {los_label}")
        ax.text(0.02, 0.95, info_text, transform=ax.transAxes, fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
        
        ax.legend(loc='upper right', fontsize=8)
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.12)

    print("✓ Dynamic trace simulation complete!")
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    create_moving_target_simulation()