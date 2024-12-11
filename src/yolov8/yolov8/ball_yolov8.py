import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int8
from ros_tools.msg import BallPos, BallPoss
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite
import numpy as np
import cv2
import time
import threading

OBJ_THRESH = 0.75
NMS_THRESH = 0.45
IMG_SIZE = (640, 640)

CLASSES = ('red', 'blue', 'black', 'yellow')


def filter_boxes(boxes, box_confidences, box_class_probs):
    box_confidences = box_confidences.reshape(-1)
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)

    return boxes[_class_pos], classes[_class_pos], (class_max_score * box_confidences)[_class_pos]


def nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    return np.array(keep)


def dfl(postion):
    n, c, h, w = postion.shape
    p_num = 4
    mc = c // p_num

    def softmax(data, dim):
        exps = np.exp(data - np.max(data, axis=dim, keepdims=True).repeat(data.shape[dim], axis=dim))
        return exps / np.sum(exps, axis=dim, keepdims=True)

    y = softmax(postion.reshape(n, p_num, mc, h, w), 2)
    acc_metrix = np.arange(mc).reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y


def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1] // grid_h, IMG_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]

    return np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)


def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    defualt_branch = 3
    pair_per_branch = len(input_data) // defualt_branch
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch * i]))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes, classes, scores = filter_boxes(
        np.concatenate([sp_flatten(_v) for _v in boxes]),
        np.concatenate([sp_flatten(_v) for _v in scores]),
        np.concatenate([sp_flatten(_v) for _v in classes_conf])
    )

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)


def letter_box(im, new_shape):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2

    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    return cv2.copyMakeBorder(im,
                              int(round(dh - 0.1)),
                              int(round(dh + 0.1)),
                              int(round(dw - 0.1)),
                              int(round(dw + 0.1)),
                              cv2.BORDER_CONSTANT, value=(0, 255, 0)
                              ), r, dw, dh


def draw(image, boxes, scores, classes, r, dw, dh):
    for box, score, cl in zip(boxes, scores, classes):
        left, top, right, bottom = map(int, box)
        left = int((left - dw) / r)
        top = int((top - dh) / r)
        right = int((right - dw) / r)
        bottom = int((bottom - dh) / r)
        print("%s @ (%d %d %d %d) %.3f" % (CLASSES[cl], left, top, right, bottom, score))
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (left, top - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


class YOLOv8Node(Node):

    def __init__(self):
        super().__init__('yolov8_node')

        self.publisher_ = self.create_publisher(BallPoss, '/yolov8/balls', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/front',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        self.rknn = RKNNLite()
        model_path = '/home/orangepi/ros2/25_rescue_vehicle/src/yolov8/resource/ball.rknn'
        self.rknn.load_rknn(model_path)
        ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().error('Init runtime environment failed')
            exit(ret)


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        img, r, dw, dh = letter_box(im=frame, new_shape=(IMG_SIZE[1], IMG_SIZE[0]))
        input_data = np.expand_dims(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), axis=0)
        output_data = self.rknn.inference([input_data])

        boxes, classes, scores = post_process(output_data)

        balls = []

        if boxes is not None:
            for box, score, cl in zip(boxes, scores, classes):
                center_x = (box[0] + box[2] - dw * 2) / 2
                center_y = (box[1] + box[3] - dh * 2) / 2

                ball = BallPos()
                ball.delta_x = -int(center_y)
                ball.delta_y = -int(center_x)
                ball.color = int(cl)
                balls.append(ball)

        if balls:
            # Create and publish the BallPos[] message
            ball_pos_msg = BallPoss()
            ball_pos_msg.balls = balls
            self.publisher_.publish(ball_pos_msg)

        img_p = frame.copy()
        if boxes is not None:
            draw(img_p, boxes, scores, classes, r, dw, dh)

        img_p = cv2.resize(img_p, (1280, 960))
        cv2.imshow("Detection", img_p)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    yolov8_node = YOLOv8Node()

    rclpy.spin(yolov8_node)

    yolov8_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
