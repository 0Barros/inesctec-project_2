# inesctec-project_2
This repo serves the purpose of helping creating the software for a drone demonstration for INESC TEC.

Project Timeline: May 8th – July 16th

## Sprints

- **Sprint 1: Communication & Mapping (May 8 – May 24)**
    
    - Setup the "Known Map" environment in Python (static obstacle definitions).
    - Implement UDP bridge for iPhone position streaming (local coordinate mapping).
    - Establish MAVLink/Wi-Fi telemetry link to receive real-time UAV coordinates.
        
- **Sprint 2: LoS Logic & Algorithm Integration (May 25 – June 8)**
    
    - Develop the **LoS Checker**: Geometric validation between iPhone, UAV, and obstacles.
    - Integrate the **TOPA Algorithm** to calculate optimal setpoints when LoS is obstructed.
    - Connect TOPA outputs to the **Lazy Theta*** and **B-Spline** path planner.
        
- **Sprint 3: Performance Dashboard & Analytics (June 9 – June 23)**
    
    - Build a real-time web dashboard (React/Socket.io).
    - Visualize the 2D/3D environment, including the "Signal Shadow" zones.
    - Track KPIs: LoS Availability, Jitter, and Command Latency.
        
- **Sprint 4: Validation & Field Trials (June 24 – July 8)**
    
    - Conduct physical tests in the cage using the iPhone and physical obstacles.
    - Stress-test the system with rapid user movement to evaluate reactive delay.
    - Finalize performance reports on QoS improvement.
        
- **Final Delivery (July 9 – July 16)**
    
    - Complete the Overleaf Technical Report and prepare the final demo.

- Finish LaTeX documentation (Overleaf).

Presentation rehearsal and final bug fixing.

### TOPA
This implementation leverages the TOPA Algorithm developed at INESC TEC to ensure optimal aerial network coverage by considering both user traffic and environmental geometry.
