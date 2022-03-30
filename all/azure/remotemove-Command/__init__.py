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

    x = req.params.get('x')
    y = req.params.get('y')
    degree = req.params.get('degree')

    if not (x and y and degree ):
        try:
            req_body = req.get_json()
        except ValueError:
            pass
        else:
            x = req_body.get('x')
            y = req_body.get('y')
            degree = req_body.get('degree')
           
            data = MSG_TXT 
            props={}
            #C2Dメッセージの書き出し
            props.update(messageId = "message_%d" % 3)
            props.update(correlationId = "correlation_%d" % 3)
            props.update(contentType = "application/json")
            props.update(contentEncoding = "utf-8")
            props.update(x = x)
            props.update(y = y)
            props.update(degree = degree)
            registry_manager.send_c2d_message(DEVICE_ID, data, properties=props)
            return func.HttpResponse(f"remote_move_command, {x,y,degree}. This HTTP Remotecontrol triggered function executed successfully.")
            
    else:
        return func.HttpResponse(
             "This HTTP triggered function executed successfully. Pass a name in the query string or in the request body for a personalized response.",
             status_code=400
        )
    
if __name__ == '__main__':
    print ( "Starting the Python IoT Hub C2D Messaging service sample..." )

    main()