import paho.mqtt.client as mqtt
import numpy as np
import json
from keras.models import load_model
import tensorflow as tf
from tensorflow.python.keras.backend import set_session

session = tf.compat.v1.Session(graph=tf.compat.v1.Graph())
MODEL_NAME = "flowers.hd5"
dict={0: 'daisy', 1: 'dandelion', 2: 'roses', 3: 'sunflowers', 4: 'tulips'}
classes = ["daisy", "dandelion", "roses", "sunflowers", "tulips"]

with session.graph.as_default():
    set_session(session)
    model = load_model(MODEL_NAME)

def classify(model, image):

    with session.graph.as_default():
        set_session(session)
        result = model.predict(image)
        themax = np.argmax(result)
    return (dict[themax], result[0][themax], themax)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Successfully connected to broker")
        client.subscribe("Group99/IMAGE/classify")
    else:
        print("Connection failed with code: %d" %rc)

def classify_flower(filename, data):
    print("Start classifying")
    try:
        label, prob, themax = classify(model, data)
    except Exception as e: 
        print("Error: ", e)
        return
    print("Done.")
    print(label, prob, themax)
    return {"filename": filename, "prediction": label, "score": str(prob), "index": str(themax)}

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

def setup(hostname):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(hostname)
    client.loop_start()
    return client

def main():
    client = setup("192.168.50.190")
    while True: 
        pass

if __name__ == "__main__":
    main()