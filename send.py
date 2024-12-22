#!/usr/bin/env python3

from requests_toolbelt.multipart.encoder import MultipartEncoder
import requests
import random
import string

def generate_random_string(length=16):
    # Генерируем случайную строку из букв и цифр
    characters = string.ascii_letters + string.digits
    random_string = ''.join(random.choice(characters) for _ in range(length))
    return random_string


host="192.168.2.58"
origh=f"http://{host}"
ref=f"{origh}/up?"
url=f"{origh}/u2"

filename="tasmota-minimal.bin.gz"
dirname="build_output/firmware/"
fullfilename=f"{dirname}{filename}"

print("Firmware update sender")

fd=open(fullfilename,"rb")
fd.seek(0,2)
fsize=fd.tell()
fd.seek(0,0)

#print(data)
uri=f"{url}?fsz={fsize}"
print(uri)

boundary = f"----WebKitFormBoundaryz{generate_random_string()}"
m = MultipartEncoder(
    fields={
        'u2': (filename, fd, 'application/gzip'),
        },
    boundary=boundary
    )

heads={
        'Upgrade-Insecure-Requests': '1',
        'Content-Type': m.content_type,
        'Host': host,
        'Origin': origh,
        'Referer': ref
}

#print(m.to_string())
response = requests.get(ref)
print(response.request.headers)
print(response.request.body)
print(response)


response = requests.post(uri, data=m, headers=heads)
print(dir(response.request))
print(response.request.headers)
print(response.request.body)
print(response)

#http://192.168.2.58/u2?fsz=264562
