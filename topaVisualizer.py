import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from topa import TOPAPositioner
from mapConfig import fligh_zone

def run_visual_test():
    """
    TOPA Engine Verification with Multiple Test Scenarios
    Tests the TOPA planner across all flight zones with varying UAV start positions.
    Edge node (generic device) is always positioned outside the cage to verify LoS recovery.
    """
    # 1. Instantiate the TOPA Engine with standard properties
    engine = TOPAPositioner(frequency_hz=5250e6, noise_floor_dbm=-85.0, max_pt_dbm=25.0)
    
    # 2. Physical parameters
    uav_radius = 0.65  # Physical radius of drone (internal to TOPA)
    
    # 3. Define test scenarios: different UAV starting positions for each zone
    test_scenarios = [
        {"label": "Corner Start", "position_offset": (0.5, 0.5)},
        {"label": "Center Start", "position_offset": (0.5, 0.5)},  # Will be adjusted to center
        {"label": "Opposite Corner", "position_offset": (0.9, 0.9)},
    ]
    
    zones = list(fligh_zone.keys())
    num_zones = len(zones)
    num_scenarios = len(test_scenarios)
    
    # Create a large figure with rows for zones and columns for scenarios
    fig, axes = plt.subplots(num_zones, num_scenarios, figsize=(18, 5 * num_zones))
    if num_zones == 1:
        axes = axes.reshape(1, -1)
    
    # 4. Run TOPA on each flight zone and scenario combination
    for zone_idx, zone_name in enumerate(zones):
        zone_config = fligh_zone[zone_name]
        
        # Extract cage limits
        raw_cage = {
            "limit_x": zone_config["limit_x"],
            "limit_y": zone_config["limit_y"],
            "limit_z": 20.0  # Default height limit
        }
        
        # Convert obstacles from map format to TOPA format
        raw_obstacles = []
        for obs in zone_config["obstacles"]:
            obs_dict = {
                "name": obs["name"],
                "x": obs["position"][0],
                "y": obs["position"][1],
                "radius": obs["radius"],
                "height": 3.0  # Default pillar height
            }
            raw_obstacles.append(obs_dict)
        
        # Define edge node position (always outside the cage, to the right)
        edge_node_pos = (raw_cage["limit_x"] + 2.0, raw_cage["limit_y"] / 2.0, 0.5)
        
        # Run each scenario
        for scenario_idx, scenario in enumerate(test_scenarios):
            ax = axes[zone_idx, scenario_idx]
            
            # Calculate UAV start position based on scenario
            if scenario_idx == 0:  # Corner start
                uav_start_x = 0.5
                uav_start_y = 0.5
            elif scenario_idx == 1:  # Center start
                uav_start_x = raw_cage["limit_x"] / 2.0
                uav_start_y = raw_cage["limit_y"] / 2.0
            else:  # Opposite corner
                uav_start_x = raw_cage["limit_x"] - 0.5
                uav_start_y = raw_cage["limit_y"] - 0.5
            
            uav_current_pos = (uav_start_x, uav_start_y, 5.0)
            
            # Prepare TOPA inputs
            safe_cage = raw_cage.copy()
            safe_obstacles = []
            for obs in raw_obstacles:
                inflated_obs = obs.copy()
                inflated_obs["radius"] = obs["radius"] + uav_radius
                safe_obstacles.append(inflated_obs)
            
            # Execute TOPA
            optimal_destination = engine.compute_topa_optimal_position(
                uav_current_pos, edge_node_pos, safe_obstacles, safe_cage
            )
            
            # =====================================================================
            # PLOTTING
            # =====================================================================
            ax.set_xlim(-2, raw_cage["limit_x"] + 4)
            ax.set_ylim(-2, raw_cage["limit_y"] + 2)
            
            # Plot cage boundaries
            cage_rect = plt.Rectangle((0, 0), raw_cage["limit_x"], raw_cage["limit_y"], 
                                      fill=False, color="red", linestyle="--", linewidth=2.0)
            ax.add_patch(cage_rect)
            ax.text(raw_cage["limit_x"] / 2, -1.2, "Cage Boundary", ha='center', 
                   color='red', fontweight='bold', fontsize=9)
            
            # Plot obstacles
            for obs in raw_obstacles:
                obs_circle = plt.Circle((obs["x"], obs["y"]), obs["radius"], 
                                       color="darkgray", alpha=0.8, edgecolor="black", linewidth=1.5)
                ax.add_patch(obs_circle)
                ax.text(obs["x"], obs["y"], obs["name"], ha='center', va='center', 
                       color='white', fontsize=7, fontweight='bold')
            
            # Plot UAV start position
            ax.plot(uav_current_pos[0], uav_current_pos[1], 'o', color='blue', markersize=10, label='UAV Start')
            uav_hull_start = plt.Circle((uav_current_pos[0], uav_current_pos[1]), uav_radius, 
                                       color="blue", fill=False, linestyle=':', linewidth=1.5, alpha=0.7)
            ax.add_patch(uav_hull_start)
            
            # Plot edge node (outside cage)
            ax.plot(edge_node_pos[0], edge_node_pos[1], 's', color='red', markersize=11, label='Edge Node')
            ax.text(edge_node_pos[0] + 0.3, edge_node_pos[1] + 0.3, "Edge Node", color='red', 
                   fontweight='bold', fontsize=8)
            
            # Plot TOPA result
            opt_x, opt_y, opt_z = optimal_destination
            ax.plot(opt_x, opt_y, 'o', color='green', markersize=12, label='TOPA Goal')
            uav_hull_dest = plt.Circle((opt_x, opt_y), uav_radius, 
                                      color="green", fill=True, alpha=0.25, edgecolor="darkgreen", linewidth=1.5)
            ax.add_patch(uav_hull_dest)
            
            # Plot line-of-sight vector
            ax.plot([opt_x, edge_node_pos[0]], [opt_y, edge_node_pos[1]], 
                   color="green", linestyle="-", linewidth=2.5, alpha=0.7, label='LoS Vector')
            
            # Plot trajectory line from start to goal
            ax.plot([uav_current_pos[0], opt_x], [uav_current_pos[1], opt_y], 
                   color="cyan", linestyle="--", linewidth=1.5, alpha=0.6, label='Planned Path')
            
            # Labels and formatting
            ax.set_title(f"{zone_config['name']} - {scenario['label']}\n(Z={opt_z:.2f}m)", 
                        fontsize=11, fontweight='bold', pad=10)
            ax.set_xlabel("X (meters)", fontsize=9)
            ax.set_ylabel("Y (meters)", fontsize=9)
            ax.grid(True, linestyle=":", alpha=0.3)
            ax.set_aspect('equal', adjustable='box')
    
    # =====================================================================
    # UNIFIED LEGEND (shown once at bottom)
    # =====================================================================
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='UAV Start Position'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10, label='Edge Node (Outside Cage)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='TOPA Optimal Goal'),
        Line2D([0], [0], color='red', linestyle='--', linewidth=2.0, label='Cage Boundary'),
        Line2D([0], [0], color='darkgray', linewidth=2.0, label='Obstacles'),
        Line2D([0], [0], color='green', linestyle='-', linewidth=2.5, label='Line-of-Sight Vector'),
        Line2D([0], [0], color='cyan', linestyle='--', linewidth=1.5, label='Planned Trajectory'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=7, bbox_to_anchor=(0.5, -0.02), fontsize=10)
    
    fig.suptitle("TOPA Engine Verification Across Flight Zones & Start Positions", 
                fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout(rect=[0, 0.03, 1, 0.99])
    plt.show()

if __name__ == "__main__":
    run_visual_test()
    plt.show()

if __name__ == "__main__":
    run_visual_test()