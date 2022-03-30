from email import message
import logging
import json
import azure.functions as func
import urllib.request, json
import requests

def main(context,req: func.HttpRequest,todoitems: func.DocumentList) -> func.HttpResponse:
    logging.info('Python HTTP trigger function processed a request.')

    if not todoitems:
        logging.warning("ToDo item not found")
    else:
        logging.info("Found ToDo item, Description=%s",
                     todoitems[0]['id'])

        logging.info("Found ToDo item, Description=%s",
                     todoitems[0]['Position_x'])
        logging.info("Found ToDo item, Description=%s",
                     todoitems[0]['Robot_ID'])

        Robot_ID = todoitems[0]['Robot_ID']           
        if (Robot_ID== "001"):
            x1 = todoitems[0]['Position_x']
            y1 = todoitems[0]['Position_y']
        elif (Robot_ID== "002"):
            x2 = todoitems[0]['Position_x']
            y2 = todoitems[0]['Position_y']

    message = {"device_list": [{"device_id": "ROBO_2", "location": [35, 139], "location_name": "base station 1", "battery_life": 47, "status": "safe"}, {"device_id": "00120202948", "location": [35.8783, 140.05419], "location_name": "base station 1", "battery_life": 233, "status": "safe"}, {"device_id": "gouldan", "location": [0, 0], "location_name": "presidents office", "battery_life": 88, "status": "safe"}, {"device_id": "tROBOlmeyka", "location": [35.652973675286006, 139.74433422088623], "location_name": "presidents office", "battery_life": 22, "status": "safe"}, {"device_id": "RoboDummy", "location": [139.7442825883627, 35.652557669457146], "location_name": "Office of Corporate Auditors", "battery_life": 22, "status": "safe"}, {"device_id": "temi_1", "location": [x1, y1], "location_name": "reception", "battery_life": 84, "status": "safe"}]}
    return func.HttpResponse(
        json.dumps(message),
        mimetype="application/json",
    )
