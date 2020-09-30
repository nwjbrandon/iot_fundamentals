import paho.mqtt.client as mqtt
import json
import numpy as np

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Successfully connected to broker with rc: ", rc)
        client.subscribe("Group99/IMAGE/classify")
    else:
        print("Connection failed with code: %d" %rc)

def classify_flower(filename, data):
    print("Start classifying")
    return {"filename": "filename", "prediction": "label", "score": str(1), "index": str(1)}

def on_message(client, userdata, msg):
    recv_dict = json.loads(msg.payload)
    img_data = np.array(recv_dict["data"])
    result = classify_flower(recv_dict["filename"], img_data)
    print("Sending results: ", result)
    try:
        client.publish("Group99/IMAGE/predict", json.dumps(result))
    except Exception as e:
        print("Error: ", e)
        return

USERID = "nwjbrandon"
PASSWORD = "password"

def setup(hostname):
    client = mqtt.Client()
    client.connect(hostname, 1883, 60)
    client.username_pw_set(USERID, PASSWORD)
    client.tls_set("/home/nwjbrandon/mqtt/ca.pem", "/home/nwjbrandon/mqtt/broker.crt", "/home/nwjbrandon/mqtt/broker.key")
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()
    return client

def main():
    client = setup("192.168.50.190")
    while True: 
        pass

if __name__ == "__main__":
    main()