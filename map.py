import folium

from mapConfig import fligh_zone, selected_zone
from convertIntoCage import draw_cage_local

def map_update(coordenates_historic): 
    if not coordenates_historic:
        print("No coordinates received yet.")
        return
    
    last_coordinates = coordenates_historic[-1]
    mission = fligh_zone[selected_zone]

    if not mission:
        return

    map = folium.Map(
        location = mission["center"],
        zoom_start = 30,
        )
    
    draw_cage_local(map)

    if len(coordenates_historic) > 1:
        folium.PolyLine(
            coordenates_historic, color="blue", weight=3, opacity=0.8
        ).add_to(map)  
        
    folium.Marker(
        location = last_coordinates,
        popup = "Iphones Current Location",
        icon = folium.Icon(color="green", icon="info-sign")
    ).add_to(map)

    map.save("map.html")
    print("Map updated with the latest coordinates.")