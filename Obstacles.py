import requests 

def login(username, password):
    body = {
        "username": username,
        "password": password
    }

    response = requests.post("http://localhost:8000/api/login", json=body)
    return response.cookies


def obstacles(cookies):

	obs = requests.get("http://localhost:8000/api/missions",cookies=cookies).json()[0]['stationaryObstacles']
	
	tuple_obs = []

	for obstacles in obs:
		tuple_obs.append((obstacles['latitude'],obstacles['longitude'],obstacles['radius'],obstacles['height']))

	return tuple_obs


cookies = login('testadmin', 'testpass')

print(obstacles(cookies))