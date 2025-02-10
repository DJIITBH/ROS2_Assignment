from ultralytics import YOLO
import cv2

# Load a model
model = YOLO("yolo11n.pt")

# Train the model
# train_results = model.train(
#     data="coco8.yaml",  # path to dataset YAML
#     epochs=100,  # number of training epochs
#     imgsz=640,  # training image size
#     device="cpu",  # device to run on, i.e. device=0 or device=0,1,2,3 or device=cpu
# )

# Evaluate model performance on the validation set
# metrics = model.val()

# Perform object detection on an image
results = model("ss.png")

for result in results:
    for box in result.boxes:
        # Extract bounding box coordinates
        x1, y1, x2, y2 = box.xyxy[0].tolist()  # Top-left and bottom-right coordinates
        confidence = box.conf.item()  # Confidence score
        class_id = box.cls.item()  # Class ID
        class_name = model.names[class_id]
        print(class_id, class_name, x1, y1, x2, y2)
# print(results[0].orig_img)
results[0].show()
# cv2.imshow('dsfgf', results[0].orig_img)
# cv2.waitKey(0)

# Export the model to ONNX format
# path = model.export(format="onnx")  # return path to exported model