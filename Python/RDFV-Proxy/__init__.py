from typing import List
import logging
import json
import requests
import os

import azure.functions as func


class Generic:
    @classmethod
    def from_dict(cls, dict):
        obj = cls()
        obj.__dict__.update(dict)
        return obj


def main(event: List[func.EventHubEvent]):
    monitor_endpoint = os.environ["monitorEndpoint"]
    headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}

    try:
        event_json = event.get_body().decode('utf-8')
        event_item = json.loads(event_json, object_hook=Generic.from_dict)

        if event_item.Type == "Settings":
            logging.info(
                f'User: --> settings received from device :{event_item.DeviceName}')
            request = requests.post(f'{monitor_endpoint}/api/settings',
                                    event_json,
                                    verify=False,
                                    headers=headers)
            if request.status_code != 200:
                logging.error(
                    f'User: Settings could not be send to endpoint {monitor_endpoint}/api/settings')

                logging.error(f"Request: {event_json}")

        elif (event_item.Type == "Data"):
            logging.info(
                f'User: --> data received from device :{event_item.DeviceName}')
            request = requests.post(f'{monitor_endpoint}/api/data',
                                    event_json,
                                    verify=False,
                                    headers=headers)
            if request.status_code != 200:
                logging.error(
                    f'User: Settings could not be send to endpoint {monitor_endpoint}/api/data')

                logging.error(f"Request: {event_json}")

        else:
            logging.error(
                f'User: --> unkown type received :{event_item.device}')

    except Exception as inst:
        logging.info(f"Error: {inst}")
