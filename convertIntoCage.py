import folium
import numpy as np

from mapConfig import fligh_zone, selected_zone

def meter_to_gps(lat_center, lon_center, limit_x, limit_y):
    meters_per_degree_lat = 111132
    meters_per_degree_lon = 111132 * np.cos(np.radians(lat_center))

    deg_lat = limit_y / meters_per_degree_lat
    deg_lon = limit_x / meters_per_degree_lon

    return [lat_center + deg_lat, lon_center + deg_lon]

def draw_cage_local(map):
    # Visualization should show the actual physical cage and obstacle sizes.
    # The TOPA planner applies the 0.65 m UAV safety margin internally, but map display stays uninflated.
    mission = fligh_zone[selected_zone]
    if not mission:
        print(f"'{selected_zone}' not found.")
        return
        
    lat_center = mission["center"][0]
    lon_center = mission["center"][1]
    limit_x = mission["limit_x"]
    limit_y = mission["limit_y"]

    x_half = limit_x / 2
    y_half = limit_y / 2
    
    corner_sup_left = meter_to_gps(lat_center, lon_center, -x_half, y_half)
    corner_sup_right = meter_to_gps(lat_center, lon_center, x_half, y_half)
    corner_inf_left = meter_to_gps(lat_center, lon_center, -x_half, -y_half)
    corner_inf_right = meter_to_gps(lat_center, lon_center, x_half, -y_half)

    lat_origin = corner_sup_left[0]
    lon_origin = corner_sup_left[1]

    folium.Polygon(
        locations=[corner_sup_left, corner_sup_right, corner_inf_right, corner_inf_left],
        color="black",
        weight=3,
        fill=True,
        fill_color="black",
        fill_opacity=0.5,
        popup="Cage Boundary"
    ).add_to(map)
    
    for obs in mission["obstacles"]:

        x_local = obs["position"][0]
        y_local = obs["position"][1]
        obs_gps = meter_to_gps(lat_origin, lon_origin, x_local, -y_local)
        
        if obs["type"] == "cylinder":
            folium.Circle(
                location=obs_gps,
                radius=obs["radius"],
                color="red",
                fill=True,
                fill_color="red",
                fill_opacity=0.4,
                popup=obs["name"]
            ).add_to(map)