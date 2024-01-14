from flask import Flask, render_template, request,  make_response, jsonify, Response
import json
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from flask_socketio import SocketIO
import cv2
import base64
from threading import Event
import queue
import time

image_queue = queue.Queue(maxsize=120)
IMAGE_TOPIC='/image'
ITEM_TOPIC='/item'

item_pub = rospy.Publisher(ITEM_TOPIC, String, queue_size=10)
class Subscriber:
    def __init__(self):
        rospy.init_node('video_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
    
    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            image_queue.put(image)
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
        classes = []
        if int(data) != 100:
            classes.append(int(data))
        print(classes)
        item_pub.publish(json.dumps(classes))
        return make_response(jsonify(message="OK"), 200)

    return render_template('index.html', items = items)
    
def gen_frames():   
    while True:
        try:
            frame = image_queue.get()  # read the camera frame
            _, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
        except:
            return "error"

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    subscriber = Subscriber()
    app.run(debug=True)
    rospy.spin()
