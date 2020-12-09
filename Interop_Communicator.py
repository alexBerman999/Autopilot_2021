import requests
class Interop_Communicator:
	
	def __init__(self):
		self.cookies = login('testadmin','testpass')

	def waypoint(self):

		wps = requests.get("http://localhost:8000/api/missions",cookies=self.cookies).json()[0]['waypoints']
	
		tuple_wps = []

		for waypoint in wps:
			tuple_wps.append((waypoint['latitude'],waypoint['longitude'],waypoint['altitude']))

		return tuple_wps

	def send_telemetry(self,latitude,longitude,altitude,heading):
		body = {
			"latitude": latitude,
			"longitude": longitude,
			"altitude": altitude,
			"heading": heading,
		}


		send = requests.post("http://localhost:8000/api/telemetry", cookies=self.cookies,json=body)
		return send





def login(username, password):
	body = { "username": username, "password": password }

	response = requests.post("http://localhost:8000/api/login", json=body)
	return response.cookies

