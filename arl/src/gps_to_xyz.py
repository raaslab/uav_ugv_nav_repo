#!/usr/bin/env python
import math

class GPS2XY:
    def __init__(self, origin_lat, origin_lon):
        self.origin_lat = math.radians(origin_lat)  # Convert degrees to radians
        self.origin_lon = math.radians(origin_lon)

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        # Radius of the Earth in meters
        R = 6371000.0 
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
             math.cos(lat1) * math.cos(lat2) * 
             math.sin(dlon / 2) * math.sin(dlon / 2))
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def convert(self, lat, lon):
        lat = math.radians(lat)
        lon = math.radians(lon)
        
        # Calculate the distance in the x and y directions
        y = self.haversine_distance(self.origin_lat, self.origin_lon, lat, self.origin_lon)
        x = self.haversine_distance(self.origin_lat, self.origin_lon, self.origin_lat, lon)
        
        # Check the direction of the x and y coordinates
        if lat < self.origin_lat:
            y = -y
        if lon < self.origin_lon:
            x = -x
        
        return x, y
    
    

# Usage:
origin = (49.9, 8.9)  # Example origin coordinates
converter = GPS2XY(*origin)

lat = 50.9
lon = 8.9
x, y = converter.convert(lat, lon)
#print("Converted GPS ({}, {}) to XY ({}, {})".format(lat, lon, x, y))
