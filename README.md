**Custom Object Detection- YOLO V5**

Yolov5 small, Custom Trained for detecting BAlls (Tennis, Golf, Tabel Tennis, Rugy, Cube) 

These are all the small and delicate items to be handeled by the Robotic arm Custom Built using Lego Mindstorm Inventor Kit, to Pick and Place the objects from one place to another by means of the depth and center points information of the objects detected. this is mainly to replicate the Manufacturing Environment without using Expensive 2D-3D lIDAR and other Sensors.

Camera: OAK-D-Lite

Operating System: Raspberry PI -4 Debian OS

Robotic Arm: Lego Mindstorm Inventor Kit


![val_batch0_pred](https://user-images.githubusercontent.com/77121467/161968990-f58149dc-6eb7-45d9-8862-3a79626d1f37.jpg)



Yolov5-small Custom TRained for People Detection.

This Model detects the people baded on the depth information from OAK-D-Lite and Robotic Arm stops working in order to have a safer zone and again here main aim is to use OAK-D-LIte camera to simulate the real-time MAnufacturing Environment with using Sensors or other Expensive Lidars 

![train_batch0](https://user-images.githubusercontent.com/77121467/161969089-c69e5df2-2695-4e66-9712-7720b6fd5242.jpg)

Annotation and Dataset Collection:

Dataset collected for BAll-dection is purely by taking photos around 200 Photos and ball Dataset has been Augumented based on Roboflow and Annotated.

Dataset Collected for People-Detection is sourced from Kaggle:  https://www.kaggle.com/datasets/constantinwerner/human-detection-dataset and its Annotated and Augumented based on Roboflow

Training and Testing Model :

Using Google Colab Tesla -GPU Instances.

Deployment :

Trained weights converted to IR-OpenVino Format and then optimized to Onnx and then Converted to Intel Myrid Blob to run on the OAK-D-Lite device.

Integration :

Integration of the Lego Mindstorm and the Oak-D-LIte has been done using the MicroPython.

Demo:

![Prototype](https://drive.google.com/file/d/1tpVU2COFiBm1LgaivJleqJ1Ldfu3xbHD/view?usp=sharing)
