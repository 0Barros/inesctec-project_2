from flask import Flask, jsonify ,request
import json

from map import map_update

coordenates_historic = []

app = Flask(__name__)

@app.route('/data', methods=['POST'])
def receive_data(): 
    global coordenates_historic
    data = request.get_json()

    if data and "payload" in data:
        for reading in data.get("payload", []):
            if reading.get("name") == "location":
                lat = reading["values"].get("latitude")
                lon = reading["values"].get("longitude")
                print(f"Received location from Iphone - Latitude: {lat}, Longitude: {lon}")

                coordenates_historic.append((lat, lon))
                map_update(coordenates_historic)

    return "Data received", 200
