import paho.mqtt.client as mqtt
import numpy as np
from PIL import Image
import json
from os import listdir
from os.path import join

SAMPLE_PATH = "./samples"

def load_image(filename):
    img = Image.open(filename)
    img = img.resize((249, 249))
    imgarray = np.array(img)/255.0
    final = np.expand_dims(imgarray, axis=0)
    return final

def send_image(client, filename):
    img = load_image(filename)
    img_list = img.tolist()
    send_dict = {"filename": filename, "data": img_list}
    client.publish("Group99/IMAGE/classify", json.dumps(send_dict))

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected")
        client.subscribe("Group99/IMAGE/predict")
    else:
        print("Failed to connect. Error code: %d." % rc)

def on_message(client, userdata, msg):
    print("Received message from server")
    resp_dict = json.loads(msg.payload)
    print(resp_dict["filename"], resp_dict["prediction"], resp_dict["score"], resp_dict["index"])

USERID = "nwjbrandon"
PASSWORD = "password"

def setup(hostname):
    client = mqtt.Client()
    client.connect(hostname, 1883, 60)
    client.username_pw_set(USERID, PASSWORD)
    client.tls_set("/home/nwjbrandon/mqtt/ca.pem", "/home/nwjbrandon/mqtt/client.crt", "/home/nwjbrandon/mqtt/client.key")
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()
    return client

def main():
    client = setup("192.168.50.190")
    print("Sending data")
    # send_image(client, "samples/tulip2.jpg")
    sample_files = listdir(SAMPLE_PATH)
    print(sample_files)
    for filename in sample_files:
        filename = join(SAMPLE_PATH, filename)
        send_image(client, filename)
    print("Done. Waiting for results.")
    while True:
        pass

if __name__ == '__main__':
    main()