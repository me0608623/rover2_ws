import pandas as pd
import numpy as np
import json
import requests
import ast

host = '140.124.42.46'
port = '8000' 

if __name__ == '__main__':
    url = 'http://{}:{}/api/mapinfo/get_map_info/'.format(host, port)
    payload = {'location': 'itc_spot', 'floor': 3}
    server_res = requests.get(url, params=payload, auth = ('rover', 'campusrover314'))
    map_info = ast.literal_eval(server_res.text)
    
    position = []
    for point in map_info['rooms']:
        position.append([point['room'], point['position']['x'], point['position']['y'], point['position']['z'], 
                    point['position']['rw'], point['position']['rx'], point['position']['ry'], point['position']['rz']])
        
    pd.DataFrame(position).to_csv("info.csv", header=0, index=0)

    all_point = np.array(position)[:, 0]
    all_connection = []
    for point in all_point:
        connection = str(point)
        for connect in map_info['connections']:
            if point in connect:
                for room in connect:
                    if room not in connection:
                        connection += ',' + str(room)  
        all_connection.append(connection)

    pd.DataFrame(all_connection).to_csv("modul.csv", header=0, index=0)

# You have to remove "" from modul.csv

