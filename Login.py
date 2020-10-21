import requests

def login(username, password):
    body = {
        "username": username,
        "password": password
    }

    response = requests.post("http://localhost:8000/api/login", json=body)
    return response.cookies


c = login('testadmin', 'testpass')

print(c)
