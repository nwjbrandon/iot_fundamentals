import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", str(rc))
    client.subscribe("hello/#")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload.decode("utf-8")))
    
USERID = "nwjbrandon"
PASSWORD = "password"
client = mqtt.Client()
client.username_pw_set(USERID, PASSWORD)
client.tls_set("/home/nwjbrandon/mqtt/ca.pem")
client.on_connect = on_connect
client.on_message = on_message

print("Connecting")
client.connect("192.168.50.190", 1883, 60)
client.loop_forever()
