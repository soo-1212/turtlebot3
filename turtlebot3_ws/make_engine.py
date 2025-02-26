from ultralytics import YOLO

model = YOLO('/home/soo/to_students/amr_best.pt')

#cpu
trt_model = model.export(format="onnx")

#gpu
#trt_model = model.export(format="engine")
