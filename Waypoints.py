import requests 

def login(username, password):
    body = {
        "username": username,
        "password": password
    }

    response = requests.post("http://localhost:8000/api/login", json=body)
    return response.cookies


def waypoint(cookies):

	wps = requests.get("http://localhost:8000/api/missions",cookies=cookies).json()[0]['waypoints']
	
	tuple_wps = []

	for waypoint in wps:
		tuple_wps.append((waypoint['latitude'],waypoint['longitude'],waypoint['altitude']))

	return tuple_wps


cookies = login('testadmin', 'testpass')

print(waypoint(cookies))