from flask import Flask, render_template, request,  make_response, jsonify
import json
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from flask_socketio import SocketIO
import cv2
import base64
from threading import Event
import queue
import time

image_queue = queue.Queue(maxsize=40)
thread_event = Event()

IMAGE_TOPIC='/image'
class Subscriber:
    def __init__(self):
        rospy.init_node('video_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
    
    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            image_queue.put(image)
            print(image_queue.qsize())
        except CvBridgeError as e:
            print(e)
            return

app = Flask(__name__, template_folder='views')
socketio = SocketIO(app)

with open("items.json") as file:
    items = json.load(file)

@app.route('/', methods=['GET', 'POST'])
def main():
    if request.method == 'POST':
        data = request.get_json()
        print(data)
        return make_response(jsonify(message="OK"), 200)

    return render_template('index.html', items = items)

def generate(event):
    while not image_queue.full():
        print("waiting")

    try:
        while event.is_set():
            print(image_queue._qsize())
            image = image_queue.get()

            _, buffer = cv2.imencode('.jpg', image)
            frame_bytes = base64.b64encode(buffer)
            socketio.emit('video_frame', {'image': frame_bytes.decode('utf-8')})
            sleep_time = 0.2
            time.sleep(sleep_time)
    finally: 
        event.clear()

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    thread_event.set()
    socketio.start_background_task(generate, thread_event)

@socketio.on('disconnect')
def handle_disconnect():
    print('dc')
    thread_event.clear()

if __name__ == '__main__':
    subscriber = Subscriber()
    app.run(debug=True)
    rospy.spin()
