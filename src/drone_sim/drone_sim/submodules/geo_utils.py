import math

def enu_to_geodetic(x, y, z, lat0, lon0, alt0):
    R = 6378137.0  

    dLat = (y / R) * (180.0 / math.pi)
    dLon = (x / (R * math.cos(math.pi * lat0 / 180.0))) * (180.0 / math.pi)
    lat = lat0 + dLat
    lon = lon0 + dLon
    alt = alt0 + z
    return lat, lon, alt

def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    R = 6378137.0  
    
    dLat = lat - lat0
    dLon = lon - lon0
    
    x = (dLon * math.pi / 180.0) * R * math.cos(math.pi * lat0 / 180.0)
    y = (dLat * math.pi / 180.0) * R
    z = alt - alt0
    
    return x, y, z
