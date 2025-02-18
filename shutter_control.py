import serial
from enum import Enum
from time import sleep
from datetime import datetime
import xml.etree.ElementTree as ET
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
import os

load_dotenv()

api_key= os.getenv("API_KEY")

mqtt_server = os.getenv("MQTT_SERVER")
mqtt_username = os.getenv("MQTT_USERNAME")
mqtt_password = os.getenv("MQTT_PASSWORD")

class Command(Enum):
    STOP = 0
    UP = 1
    DOWN = 2
    POS1 = 3
    POS2 = 4

class Device(Enum):
    WOHNZIMMER_L = {
        "id": "AQAAAAAAAAA=",
        "name": "wohnzimmer_l",
        "friendly_name": "Wohnzimmer links",
    }
    WOHNZIMMER_R = {
        "id": "AgAAAAAAAAA=",
        "name": "wohnzimmer_r",
        "friendly_name": "Wohnzimmer rechts",
    }
    SCHLAFZIMMER = {
        "id": "BAAAAAAAAAA=",
        "name": "schlafzimmer",
        "friendly_name": "Schlafzimmer",
    }
    GAESTEZIMMER = {
        "id": "CAAAAAAAAAA=",
        "name": "gaestezimmer",
        "friendly_name": "Gaestezimmer",
    }
    WOHNZIMMER = {
        "id": "EAAAAAAAAAA=",
        "name": "wohnzimmer",
        "friendly_name": "Wohnzimmer komplett",
    }

def send(device: Device, command: Command):
    commandText = f"<methodCall> \
	<methodName>selve.GW.iveo.commandManual</methodName> \
	<array> \
		<base64>{device.value['id']}</base64> \
		<int>{command.value}</int> \
	</array> \
    </methodCall>"
    ser.write(bytes(commandText, "utf-8"))

def state_topic(device_name):
    return f"homeassistant/cover/{device_name}/state"

def position_topic(device_name):
    return f"homeassistant/cover/{device_name}/position"

def open(device: Device):
    send(device, Command.UP)
    publish_open(device)
    if (device == Device.WOHNZIMMER):
        publish_open(Device.WOHNZIMMER_L)
        publish_open(Device.WOHNZIMMER_R)

def close(device: Device):
    send(device, Command.DOWN)
    publish_close(device)
    if (device == Device.WOHNZIMMER):
        publish_close(Device.WOHNZIMMER_L)
        publish_close(Device.WOHNZIMMER_R)

def stop(device: Device):
    send(device, Command.STOP)
    publish_stop(device)
    if (device == Device.WOHNZIMMER):
        publish_stop(Device.WOHNZIMMER_L)
        publish_stop(Device.WOHNZIMMER_R)

def position40(device: Device):
    send(device, Command.POS1)
    publish_position40(device)
    if (device == Device.WOHNZIMMER):
        publish_position40(Device.WOHNZIMMER_L)
        publish_position40(Device.WOHNZIMMER_R)

def position10(device: Device):
    send(device, Command.POS2)
    publish_position10(device)
    if (device == Device.WOHNZIMMER):
        publish_position10(Device.WOHNZIMMER_L)
        publish_position10(Device.WOHNZIMMER_R)

def publish_open(device: Device):
    mqttc.publish(state_topic(device.value["name"]), "open", 0, True)
    mqttc.publish(position_topic(device.value["name"]), "100", 0, True)

def publish_close(device: Device):
    mqttc.publish(state_topic(device.value["name"]), "closed", 0, True)
    mqttc.publish(position_topic(device.value["name"]), "0", 0, True)

def publish_stop(device: Device):
    mqttc.publish(state_topic(device.value["name"]), "stopped", 0, True)
    mqttc.publish(position_topic(device.value["name"]), "50", 0, True)

def publish_position40(device: Device):
    mqttc.publish(state_topic(device.value["name"]), "open", 0, True)
    mqttc.publish(position_topic(device.value["name"]), "40", 0, True)

def publish_position10(device: Device):
    mqttc.publish(state_topic(device.value["name"]), "open", 0, True)
    mqttc.publish(position_topic(device.value["name"]), "10", 0, True)

def publish_discovery():
    print(f"Status: {mqttc.is_connected()}")
    for device in Device:
        discovery = f"""
        {{
        "name":"{device.value['friendly_name']}",
        "command_topic":"homeassistant/cover/{device.value['name']}/set",
        "state_topic":"homeassistant/cover/{device.value['name']}/state",
        "availability_topic":"homeassistant/cover/selve/status",
        "position_topic":"homeassistant/cover/{device.value['name']}/position",
        "set_position_topic":"homeassistant/cover/{device.value['name']}/position/set",
        "unique_id":"{device.value['name']}_shutter",
        "device_class":"shutter",
        "device":{{
            "identifiers":[
                "selve_gateway"
            ],
            "name":"Rolladen"
        }}
        }}"""
        mqttc.publish(f"homeassistant/cover/{device.value['name']}/config", discovery)
        #mqttc.publish(f"homeassistant/cover/{device.value['name']}/position", "90")
        mqttc.subscribe(f"homeassistant/cover/{device.value['name']}/set")
        mqttc.subscribe(f"homeassistant/cover/{device.value['name']}/position/set")
    mqttc.publish("homeassistant/cover/selve/status", "online")
    mqttc.subscribe("homeassistant/status")
    print(f"{datetime.now()}discovery")

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    mqttc.subscribe("homeassistant/status")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #client.subscribe("$SYS/#")
    publish_discovery()

def on_message(client, userdata, message):
    """Handle an MQTT message."""
    topic = message.topic
    payload = message.payload.decode()

    print(f"Received message on {topic}: {payload}")

    if topic == "homeassistant/status" and payload == "online":
        publish_discovery()
        return
    elif topic.startswith("homeassistant/cover/schlafzimmer/"):
        device = Device.SCHLAFZIMMER
    elif topic.startswith("homeassistant/cover/gaestezimmer/"):
        device = Device.GAESTEZIMMER
    elif topic.startswith("homeassistant/cover/wohnzimmer_l/"):
        device = Device.WOHNZIMMER_L
    elif topic.startswith("homeassistant/cover/wohnzimmer_r/"):
        device = Device.WOHNZIMMER_R
    elif topic.startswith("homeassistant/cover/wohnzimmer/"):
        device = Device.WOHNZIMMER
    else:
        return

    if payload == "OPEN" or payload == "100":
        open(device)
    elif payload == "CLOSE" or payload == "0":
        close(device)
    elif payload == "STOP":
        stop(device)
    elif payload == "10":
        position10(device)
    elif payload == "40":
        position40(device)

    sleep(2)


### Start of main program

#ser = serial.Serial("/dev/ttyUSB0", 115200)  # open serial port
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'

ser.open()
while True:
    ser.write(
        b"<methodCall> \
            <methodName>selve.GW.service.getState</methodName> \
            </methodCall>"
    )
    sleep(0.1)
    response = ser.read(ser.in_waiting).decode("utf-8")
    root = ET.fromstring(response)
    if (root[0][0].text == "selve.GW.service.getState") & (root[0][1].text == "3"):
        print(f"Sucessfully connected to Selve Gateway on {ser.name}")
        break

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.username = mqtt_username
mqttc.password = mqtt_password


mqttc.will_set("homeassistant/cover/selve/status", "offline")
#mqttc.connect(mqtt_server)

while True:
  mqttc.loop()
  if not mqttc.is_connected():
    print(f"{datetime.now()} not connected")
    mqttc.connect(mqtt_server)
    publish_discovery()
  now = datetime.now()
  if now.minute % 10 == 0 and (now.second <2):
    #print(f"{datetime.now()} discovery")
    publish_discovery()
  sleep(0.1)

#mqttc.loop_forever()

ser.close()
