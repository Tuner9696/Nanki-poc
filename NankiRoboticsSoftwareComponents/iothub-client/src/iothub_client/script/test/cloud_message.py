import random
import sys
import json


from azure.iot.hub import IoTHubRegistryManager
from azure.iot.device import Message
from azure.iot.device import IoTHubDeviceClient


MESSAGE_COUNT = 2
AVG_WIND_SPEED = 10.0
yuto = "aa"
kuroki = "CC"
MSG_TXT = "{\"service client sent a message\": %.2f}"

#MSG_TXT ='{{"A": {yuto} ,"sss": {kuroki}}}'


CONNECTION_STRING = "HostName=iot-nec-dev.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=ZL3yUr0Jqt7Rr0oLMclfSwPHU8nBGzw0A4zXcz0O50w="
DEVICE_ID = "thermostat67"

def iothub_messaging_sample_run():
    
        # Create IoTHubRegistryManager
        registry_manager = IoTHubRegistryManager(CONNECTION_STRING)
     
        while True:

            data = MSG_TXT 
  
            props={}
            # optional: assign system properties
            
            props.update(messageId = "message_%d" % 3)
            #props.remove(messageId )
            props.update(correlationId = "correlation_%d" % 3)
            props.update(contentType = "application/json")
            props.update(contentEncoding = "utf-8")

            # optional: assign application properties
            #mission_trigger = True
            mission_trigger = True
            remote_trigger = True   
            x = random.randrange(10, 50)
            y = random.randrange(10, 50)
            degree = random.randrange(10, 50)
 
    
            props.update(mission_trigger = mission_trigger)
            props.update(remote_trigger = remote_trigger)
            props.update(x = x)
            props.update(y = y)
            props.update(degree = degree)

            registry_manager.send_c2d_message(DEVICE_ID, data, properties=props)
            


if __name__ == '__main__':
    print ( "Starting the Python IoT Hub C2D Messaging service sample..." )

    iothub_messaging_sample_run()