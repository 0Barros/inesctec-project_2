import numpy as np
import matplotlib.pyplot as plt
import time
from topa import TOPAPositioner
from mapConfig import fligh_zone  # Leverages your actual map assets

def create_moving_target_simulation():
    # 1. Initialize TOPA engine with standard parameters
    engine = TOPAPositioner(frequency_hz=5250e6, noise_floor_dbm=-85.0, max_pt_dbm=25.0)
    
    # 2. Extract your exact B_Auditory flight zone properties from mapConfig
    zone_key = "B_Auditory"
    zone = fligh_zone[zone_key]
    
    # Strict project dimensions: 8m (X) x 4m (Y) x 3m (Z)
    cage_config = {
        "limit_x": float(zone.get("limit_x", 8.0)),   
        "limit_y": float(zone.get("limit_y", 4.0)),   
        "limit_z": 3.0  # Explicitly setting Z to 3m ceiling
    }
    
    # 3. Import ALL actual obstacles defined in your mapConfig file
    obstacles_list = []
    for obs in zone["obstacles"]:
        x, y = obs["position"]
        radius = obs["radius"]  # Raw physical radius (e.g., 0.5m)
        obstacles_list.append({
            "x": float(x),
            "y": float(y),
            "radius": float(radius),
            "height": 4.0  
        })
    
    # 4. FIXED INITIAL POSITION: Force the UAV to start on the far LEFT side
    # It will start safely inside the cage at X=0.8m, Y=2.0m
    uav_current_pos = [0.8, 2.0, 1.5]
    
    # Drone maximum speed step per frame (in meters) to prevent instant teleportation
    uav_speed_step = 0.35 
    
    # 5. DYNAMIC TARGET: Animate the EdgeNode moving OUTSIDE the cage over 50 frames
    # It moves along the outside right wall from Y=0.5 to Y=3.5 at a distance X=11.0
    num_frames = 50
    edge_node_y_trajectory = np.linspace(0.5, 3.5, num_frames)
    edge_node_x = 11.0  # Outside the 8m wide cage
    
    constant_snr_demand = 16.0 
    
    # Setup matplotlib interactive animation window
    plt.ion()
    fig, ax = plt.subplots(figsize=(11, 6))
    
    print("=" * 80)
    print(f"RUNNING DYNAMIC TRANSIT & LOS INTERSECTION SIMULATION - {zone['name']}")
    print("=" * 80)
    
    for frame_idx in range(num_frames):
        ax.clear()
        
        # Update target position for this specific frame
        current_target_y = edge_node_y_trajectory[frame_idx]
        edge_node_pos = (edge_node_x, current_target_y, 1.0)
        edge_node_with_snr = (edge_node_pos[0], edge_node_pos[1], edge_node_pos[2], constant_snr_demand)
        
        # Calculate TOPA's mathematical optimal target destination point
        optimal_pos = engine.compute_topa_optimal_position(
            uav_current_pos, edge_node_with_snr, obstacles_list, cage_config)
        
        # --- DRONE TRANSIT KINEMATICS LOOP ---
        # Instead of teleporting, move the current drone position incrementally 
        # toward the optimal target destination
        dx = optimal_pos[0] - uav_current_pos[0]
        dy = optimal_pos[1] - uav_current_pos[1]
        dist_to_optimal = np.sqrt(dx**2 + dy**2)
        
        if dist_to_optimal > uav_speed_step:
            # Step incrementally along the path vector
            uav_current_pos[0] += (dx / dist_to_optimal) * uav_speed_step
            uav_current_pos[1] += (dy / dist_to_optimal) * uav_speed_step
            # For this 2D project view, update Z directly to what TOPA requests
            uav_current_pos[2] = optimal_pos[2]
        else:
            # If close enough, settle onto the optimal point
            uav_current_pos = list(optimal_pos)
            
        opt_x, opt_y, opt_z = uav_current_pos
        
        # --- Real-Time Line of Sight (LoS) Intersection Verification Check ---
        # Draw a mathematical segment between the drone's current position and the EdgeNode
        # to determine if an obstacle cuts the ray block.
        los_is_blocked = False
        uav_to_target_dist = np.sqrt((opt_x - edge_node_pos[0])**2 + (opt_y - edge_node_pos[1])**2)
        
        for obs in obstacles_list:
            # Vector math checking intersection with the raw physical cylinder
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

        # Assign color code based on visual link validation state
        los_color = 'red' if los_is_blocked else 'lime'
        los_label = 'Line-of-Sight BLOCKED (No LoS)' if los_is_blocked else 'Line-of-Sight CLEARED (Active LoS)'
        
        # Calculate real-time clearance margins to nearest obstacle skin
        min_dist_to_obstacle = float('inf')
        for obs in obstacles_list:
            dist = np.sqrt((opt_x - obs["x"])**2 + (opt_y - obs["y"])**2) - obs["radius"]
            min_dist_to_obstacle = min(min_dist_to_obstacle, dist)
        cspace_margin = min_dist_to_obstacle - engine.uav_radius
        
        # --- GRAPHICS CANVAS RENDERING ENGINE ---
        ax.set_xlim(-1, edge_node_x + 2)
        ax.set_ylim(-1, cage_config["limit_y"] + 1.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2)
        
        # Draw the 8x4 Physical Cage Boundary (Red dashed box)
        rect = plt.Rectangle((0, 0), cage_config["limit_x"], cage_config["limit_y"], 
                            fill=False, edgecolor='red', linestyle='--', linewidth=2.5, 
                            label=f"Flight Cage Boundary ({int(cage_config['limit_x'])}mx{int(cage_config['limit_y'])}m)")
        ax.add_patch(rect)
        
        # Draw ALL obstacles from your map config file
        for i, obs in enumerate(obstacles_list):
            # Physical pillar structural skin
            ax.add_patch(plt.Circle((obs["x"], obs["y"]), obs["radius"], color='gray', alpha=0.8, zorder=3))
            # C-Space safety boundary zone (Orange dotted)
            cspace_r = obs["radius"] + engine.uav_radius
            cspace_circle = plt.Circle((obs["x"], obs["y"]), cspace_r, fill=False, 
                                      edgecolor='orange', linestyle=':', linewidth=2, zorder=2)
            ax.add_patch(cspace_circle)
            if i == 0:
                cspace_circle.set_label('C-Space Danger Zone (0.65m Buffer)')
        
        # Draw Moving Target EdgeNode OUTSIDE the cage
        ax.plot(edge_node_pos[0], edge_node_pos[1], 'rs', markersize=12, label='EdgeNode (Moving Target)', zorder=5)
        
        # Draw TOPA Calculated destination target point (where the drone wants to go)
        ax.plot(optimal_pos[0], optimal_pos[1], 'bx', markersize=8, label='TOPA Target Goal', zorder=5)
        
        # Draw current active UAV center position marker and physical 0.65m hull space
        ax.plot(opt_x, opt_y, 'go', markersize=10, label='UAV Current Center', zorder=6)
        ax.add_patch(plt.Circle((opt_x, opt_y), engine.uav_radius, fill=True, color='green', alpha=0.3, label='UAV 0.65m Hull Space', zorder=4))
        
        # Draw the Line-of-Sight Link Vector with dynamic status color rendering
        ax.plot([opt_x, edge_node_pos[0]], [opt_y, edge_node_pos[1]], color=los_color, linewidth=2.5, zorder=1, label=los_label)
        
        # Heads-Up Display (HUD) Readout Panels
        ax.set_title(f'TOPA Dynamic Tracker: {zone["name"]} Loop (Frame {frame_idx+1}/{num_frames})', fontsize=11, fontweight='bold')
        info_text = (f"Target Position: ({edge_node_pos[0]:.2f}, {edge_node_pos[1]:.2f})\n"
                    f"Drone Position:  ({opt_x:.2f}, {opt_y:.2f})\n"
                    f"Calculated Altitude Z: {opt_z:.2f} m\n"
                    f"Distance to Target: {uav_to_target_dist:.2f} m\n"
                    f"C-Space Clearance:  {cspace_margin:.2f} m\n"
                    f"Link State: {los_label.split('(')[0]}")
        ax.text(0.02, 0.95, info_text, transform=ax.transAxes, fontsize=9, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
        
        ax.legend(loc='lower left', fontsize=8)
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.12)  # Controlled step speed for smooth tracking observation
        
    print("✓ Dynamic trace simulation complete!")
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    create_moving_target_simulation()