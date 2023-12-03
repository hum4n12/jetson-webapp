# jetson-webapp

## In order to make this repository working you need to setup jetson-yolov8 object detection repository

Simple python server written in Flask to stream modified video with detected object to client browser and make a possibility for client to choose which object should be detected

<b>Python version == 3.8.10</b>

After jetson-yolov8 setup, start server by typing:
<b>python main.py</b>

and go to localhost:5000 page in the browser.

Currently, user canno choose object to detect and video stream has very poor quality, will fixup in the future