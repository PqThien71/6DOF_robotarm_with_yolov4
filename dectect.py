
import cv2
import numpy as np
import time


def get_output_layers(net):  # lấy các layout trong output mạng
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers


classes = None  # tạo class từ file yolo.names
with open('config/yolo.names', 'r') as f:
    classes = [line.strip() for line in f.readlines()]


def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):  # vẽ các BBox

    label = str(classes[class_id])
    COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

    color = COLORS[class_id]

    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)

    cv2.putText(img, label, (x - 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText(img, label, (x - 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def detection(frame):
    Width = frame.shape[1]
    Height = frame.shape[0]
    scale = 0.00392

    net = cv2.dnn.readNet(
        "config/yolov4-4k.weights", "config/yolov4-custom.cfg")

    blob = cv2.dnn.blobFromImage(
        frame, scale, (416, 416), (0, 0, 0), True, crop=False)
    # hàm blob biến hình ảnh thành một đốm màu
    # image : đầu vào / scale: hệ số tỉ lệ để biến từ 1..255 thành 0..1 / kích thước ảnh vuông/ swap RB
    net.setInput(blob)

    outs = net.forward(get_output_layers(net))

    class_ids = []  # class id
    confidences = []  # ngưỡng chính xác
    boxes = []  # bbox
    conf_threshold = 0.5
    nms_threshold = 0.4

    # Thực hiện xác định bằng HOG và SVM
    start = time.time()

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * Width)
                center_y = int(detection[1] * Height)
                w = int(detection[2] * Width)
                h = int(detection[3] * Height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h, center_x, center_y])

    indices = cv2.dnn.NMSBoxes(
        boxes, confidences, conf_threshold, nms_threshold)
    listSanPham = []
    for i in indices:
        i = i[0]
        box = boxes[i]
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        center_x = box[4]
        center_y = box[5]
        listSanPham.append(str(classes[class_ids[i]]) + ":" + str(center_x)+":" +
                           str(center_y))
        draw_prediction(frame, class_ids[i], confidences[i], round(
            x), round(y), round(x + w), round(y + h))
    return (listSanPham)
