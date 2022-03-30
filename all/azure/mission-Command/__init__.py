import logging
import json

import random
import sys
import json

import azure.functions as func
from azure.iot.hub import IoTHubRegistryManager
from azure.iot.device import Message
from azure.iot.device import IoTHubDeviceClient

MSG_TXT = "{\"service client sent a message\": %.2f}"
CONNECTION_STRING = "HostName=NS-IoTHub.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=lxoxL9QQEShP37bLfBYO4SW82bcG7mVNtMu+LQ3WZf8="
DEVICE_ID = "NS-MEC"
            
def main(req):
    logging.info('Python HTTP trigger function processed a request.')
    registry_manager = IoTHubRegistryManager(CONNECTION_STRING)
    mission = req.params.get('mission')
      
    if not mission:
        try:
            req_body = req.get_json()
        except ValueError:
            pass
        else:    
            mission = req_body.get('mission')
            data = MSG_TXT 
            props={}
            #C2Dメッセージの書き出し
            props.update(messageId = "message_%d" % 3)
            props.update(correlationId = "correlation_%d" % 3)
            props.update(contentType = "application/json")
            props.update(contentEncoding = "utf-8")
            props.update(mission = mission)
            registry_manager.send_c2d_message(DEVICE_ID, data, properties=props)    
            return func.HttpResponse(f"mission_command {mission}. This HTTP Mission triggered function executed successfully.") 
     
    else:
        return func.HttpResponse(
             "This HTTP triggered function executed successfully. Pass a name in the query string or in the request body for a personalized response.",
             status_code=400
        )
        
if __name__ == '__main__':
    print ( "Starting the Python IoT Hub C2D Messaging service sample..." )

    main()


