

from roboflow import Roboflow
rf = Roboflow(api_key="zIH1xZbDAKnDSb2cXv77")
project = rf.workspace("california-state-university-east-bay-wkf0d").project("sea_slug_objectdetection")
version = project.version(2)
dataset = version.download("yolov8")
                