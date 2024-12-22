#!/usr/bin/env python3

from requests_toolbelt.multipart.encoder import MultipartEncoder
import requests
import random
import string
import time

def generate_random_string(length=16):
    # Генерируем случайную строку из букв и цифр
    characters = string.ascii_letters + string.digits
    random_string = ''.join(random.choice(characters) for _ in range(length))
    return random_string


host="192.168.2.58"

def sendupdatetasmota(host,filename):
    dirname="build_output/firmware/"
    fullfilename=f"{dirname}{filename}"
    fd=open(fullfilename,"rb")
    fd.seek(0,2)
    fsize=fd.tell()
    fd.seek(0,0)
    print(f"Send file {filename} size {fsize}....")
    origh=f"http://{host}"
    ref=f"{origh}/up?"
    url=f"{origh}/u2"
    uri=f"{url}?fsz={fsize}"
    while True:
        try:
            response = requests.get(ref)
            if response.status_code == 200:
                break
        except:
            print("Wait device...")
        print("And wait device...")
        time.sleep(1)
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
    response = requests.post(uri, data=m, headers=heads)
    print(dir(response.request))
    print(response.request.headers)
    print(response.request.body)
    print(response)


print("Firmware update sender")
sendupdatetasmota(host,"tasmota-minimal.bin.gz")
time.sleep(10)
sendupdatetasmota(host,"tasmota-sensors.bin.gz")
