## Object detection sample code using YOLO
This code uses the code by [eriklindernoren](https://github.com/eriklindernoren/PyTorch-YOLOv3).
Please refer to the following link for more detailed information.

https://github.com/eriklindernoren/PyTorch-YOLOv3

From this link, you can learn how to train the network, replace the weights, etc.

### Install requirements

After you install all the packages related to pytorch and torchvision, you should additionally install the libraries by following commands
```
$ cd ~/capstone2_ROS_gazebo
$ sudo pip3 install -r requirements.txt
```

### Download pretrained weights
```
$ cd ~/capstone2_ROS_gazebo/src/object_detection/weights/
$ bash download_weights.sh
```

*Tested in pytorch 1.6*

### Execution
```
    $ rosrun object_detection yolo_detection.py
```
topic name : /yolo_result

### Troubleshooting

- Deactivate conda by following command
    $ conda deactivate

### Contact
Jonghwi Kim, stkimjh@kaist.ac.kr
SungHoon Yoon, yoon307@kaist.ac.kr

