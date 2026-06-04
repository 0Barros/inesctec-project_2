import numpy as np
from scipy.optimize import minimize

class TOPAPositioner:
    """
    Traffic- and Obstacle-aware UAV Positioning Algorithm (TOPA) engine.
    Optimizes UAV 3D placement to minimize overall path loss/capacity while
    guaranteeing line-of-sight (LoS) clearance over cylindrical structures
    and satisfying minimum user communication/throughput metrics.
    """
    def __init__(self, frequency_hz=5250e6, noise_floor_dbm=-85.0, max_pt_dbm=20.0):
        # Speed of light
        self.c = 3.0e8 
        self.frequency = frequency_hz
        self.noise_floor = noise_floor_dbm
        self.max_pt = max_pt_dbm
        self.uav_radius = 0.65  # internal safety padding for the UAV body in meters
        
        # Precompute propagation constant K from Eq. (3) & (4)
        # K = -20*log10(f) - 20*log10(4*pi/c) - P_N
        self.K = (-20.0 * np.log10(self.frequency) 
                  - 20.0 * np.log10((4.0 * np.pi) / self.c) 
                  - self.noise_floor)

    def _check_los_and_get_required_z(self, uav_xy, target_pos, obstacles_list):
        """
        Implements the math behind theta_i1 <= theta_i2 (Eq. 1f).
        Finds the minimum altitude Z required for a UAV at coordinate (X, Y)
        to maintain line-of-sight with the target over cylindrical obstacles.
        """
        uav_x, uav_y = uav_xy
        tx, ty, tz = target_pos[:3]  # Handle 4-tuple with SNR demand at index 3
        
        max_required_z = tz # Baseline height is at least the target height
        
        # Vector from target to UAV in 2D
        dx = uav_x - tx
        dy = uav_y - ty
        dr_sq = dx**2 + dy**2
        dr = np.sqrt(dr_sq)
        
        if dr < 1e-5:
            return max_required_z

        for obs in obstacles_list:
            ox, oy = obs["x"], obs["y"]
            # Inflate the obstacle radius by the UAV footprint so the centerline clearance includes the drone body.
            r = obs["radius"] + self.uav_radius
            h = obs.get("height", 20.0) # Defaults to the paper's 20m if not explicitly passed
            
            # Distance from obstacle center to the 2D line segment between Target and UAV
            # Line equation parameterized: P(t) = Target + t * (UAV - Target), t in [0, 1]
            # Vector from Target to Obstacle center
            vox_x = ox - tx
            vox_y = oy - ty
            
            # Projection factor t
            t = (vox_x * dx + vox_y * dy) / dr_sq
            t = max(0.0, min(1.0, t)) # Clamp to line segment
            
            # Closest point on line segment to obstacle center
            closest_x = tx + t * dx
            closest_y = ty + t * dy
            
            dist_to_line = np.sqrt((closest_x - ox)**2 + (closest_y - oy)**2)
            
            # If the line segment cuts through the cylinder's radius radius footprint
            if dist_to_line < r:
                # Find the intersection points of the line with the cylinder circle to be exact
                # For safety and edge performance, we evaluate the clearance at the closest point
                dist_from_target = np.sqrt((closest_x - tx)**2 + (closest_y - ty)**2)
                
                # Similar triangles for height clearance (theta_i1 <= theta_i2)
                # (Z_clearance - Target_Z) / Total_2D_Dist = (Obs_Height - Target_Z) / Obs_2D_Dist
                if dist_from_target > 1e-3:
                    required_z_for_obs = tz + (h - tz) * (dr / dist_from_target)
                    if required_z_for_obs > max_required_z:
                        max_required_z = required_z_for_obs

        return max_required_z

    def compute_topa_optimal_position(self, uav_pos, target_pos, obstacles_list, cage_limits):
        """
        Executes Algorithm A loop logic. Scaled for a strict Local Coordinate System interface.
        
        :param uav_pos: tuple (x, y, z) in meters (current position)
        :param target_pos: tuple (x, y, z) in meters (iPhone target position)
        :param obstacles_list: list of dicts [{"name": str, "x": float, "y": float, "radius": float, "height": float}]
        :param cage_limits: dict {"limit_x": float, "limit_y": float, "limit_z": float}
        :return: optimal_uav_pos tuple (x, y, z) in meters
        """
        # Read environment boundaries from the physical cage definition.
        # Internally, contract the X/Y bounds by the UAV radius so the UAV center remains a safe distance from the walls.
        max_x = cage_limits.get("limit_x", 50.0)
        max_y = cage_limits.get("limit_y", 50.0)
        max_z = max(0.0, cage_limits.get("limit_z", 45.0) - self.uav_radius) # Safety roof for flying space
        
        # Target requirements matching standard profile (e.g., Target MCS index 1 requires ~14 dB SNR)
        target_snr = target_pos[3] if len(target_pos) > 3 else 14.0 
        
        # Step 1: Initialize transmission power to 0 dBm (Algorithm A, Line 1)
        pt = 0.0 
        optimal_coords = None

        # Step 2: Optimization loop increasing transmission power up to max allowed (Algorithm A, Line 2)
        while pt <= self.max_pt:
            
            # Calculate maximum allowable distance (radius of traffic sphere) based on current Pt
            # From Eq. (3): d_max = 10**((K + Pt - SNR) / 20)
            d_max = 10.0 ** ((self.K + pt - target_snr) / 20.0)
            
            # Objective Function: Minimize distance to keep total capacity minimal (Eq. 1a)
            # and minimize transit distance from current position
            def objective_function(xy):
                dist_to_target = np.sqrt((xy[0] - target_pos[0])**2 + (xy[1] - target_pos[1])**2)
                dist_from_current = np.sqrt((xy[0] - uav_pos[0])**2 + (xy[1] - uav_pos[1])**2)
                return dist_to_target + 0.1 * dist_from_current

            # Constraint: Must fall within the traffic sphere maximum horizontal footprint
            def traffic_sphere_constraint(xy):
                req_z = self._check_los_and_get_required_z(xy, target_pos, obstacles_list)
                # Total 3D distance square
                dist_3d_sq = (xy[0] - target_pos[0])**2 + (xy[1] - target_pos[1])**2 + (req_z - target_pos[2])**2
                return d_max**2 - dist_3d_sq

            constraints = [{'type': 'ineq', 'fun': traffic_sphere_constraint}]
            bounds = [(self.uav_radius, max_x - self.uav_radius), (self.uav_radius, max_y - self.uav_radius)]
            
            # Initial guess: midpoint between target and current UAV position
            x0 = [0.5 * (uav_pos[0] + target_pos[0]), 0.5 * (uav_pos[1] + target_pos[1])]
            
            # Run Non-linear Constraint Optimization Solver
            res = minimize(objective_function, x0, method='SLSQP', bounds=bounds, constraints=constraints)
            
            if res.success:
                opt_x, opt_y = res.x
                opt_z = self._check_los_and_get_required_z((opt_x, opt_y), target_pos, obstacles_list)
                
                # Verify final altitude does not breach maximum allowed altitude safety cage
                if opt_z <= max_z:
                    optimal_coords = (float(opt_x), float(opt_y), float(opt_z))
                    break # Valid solution found! (Algorithm A, Line 7)
            
            # Step 3: Increment transmission power by 1 dBm if bounds are unfeasible (Algorithm A, Line 9)
            pt += 1.0
            
        # Fallback Strategy: If unfeasible under capacity constraints, prioritize physical safety & LoS recovery 
        if optimal_coords == None:
            fallback_z = self._check_los_and_get_required_z((uav_pos[0], uav_pos[1]), target_pos, obstacles_list)
            optimal_coords = (uav_pos[0], uav_pos[1], min(fallback_z, max_z))

        return optimal_coords

# =================================================================
# Production Unit Validation & Verification Test Case
# =================================================================
if __name__ == "__main__":
    # Instantiate engine with paper defaults
    engine = TOPAPositioner(frequency_hz=5250e6, noise_floor_dbm=-85.0, max_pt_dbm=25.0)
    
    # Environment Setup matching Paper Scenario A (Section V / VI-B)
    current_uav = (0.0, 0.0, 10.0)
    iphone_target = (0.0, -15.0, 1.0) # Target placed at y = -15m, height = 1m
    
    # One central obstacle matching paper setup (10m diameter radius=5m, height=20m)
    obstacles = [{"name": "building_alpha", "x": 0.0, "y": -5.0, "radius": 5.0, "height": 20.0}]
    cage = {"limit_x": 100.0, "limit_y": 100.0, "limit_z": 50.0}
    
    next_position = engine.compute_topa_optimal_position(current_uav, iphone_target, obstacles, cage)
    
    print("--------------------------------------------------")
    print(" TOPA ENGINE OUTPUT VERIFICATION")
    print("--------------------------------------------------")
    print(f"Target Input Position:  {iphone_target[:3]} m")
    print(f"Calculated Optimal Safe Placement: X={next_position[0]:.2f}, Y={next_position[1]:.2f}, Z={next_position[2]:.2f} m")
    print("--------------------------------------------------")