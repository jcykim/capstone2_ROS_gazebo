
###  Modify your code

before you run testimage_publisher, Please change image directory


image_1 = cv::imread("write down youre image directory!!!!.325.jpg",CV_LOAD_IMAGE_COLOR);

image_2 = cv::imread("/"write down youre image directory!!!!/dog.232.jpg",CV_LOAD_IMAGE_COLOR);



### If you have a problem with below error.(for running "catdog_cnn_network.py")

ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type

- These errors are caused by conflicts between Ross's OpenCV(python) and Ubuntu's OpenCV(python).
So you can solve this problem by disabling ROS;s openCV(python).

```
cd /opt/ros/kinetic/lib/python2.7/dist-packages
sudo mv cv2.so cv2.so.copy
```


