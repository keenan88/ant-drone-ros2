# https://docs.ultralytics.com/modes/train/#introduction
# https://colab.research.google.com/github/ultralytics/ultralytics/blob/main/examples/tutorial.ipynb#scrollTo=kUMOQ0OeDBJG


                

# from roboflow import Roboflow
# rf = Roboflow(api_key="3Xw1sdBlNcraC9g28zGw")
# project = rf.workspace("kushal-mujral-im3gh").project("boxes-fhdnr")
# version = project.version(4)
# dataset = version.download("coco-segmentation")

from ultralytics import YOLO
# Load a model
model = YOLO("yolo11n-seg.pt")

# Use the model
results = model.train(data='./datasets/combined/data.yaml', epochs=10, cache=False)  # train the model
results = model.val()  # evaluate model performance on the validation set
results = model.export(format='onnx')  # export the model to ONNX format