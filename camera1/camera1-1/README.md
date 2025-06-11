# Jetson 카메라 영상을 PC로 전송하여 출력하는 패키지

➨ 본 시스템은 Jetson에서 실시간으로 영상을 받고, GStreamer를 통해 PC로 전송하는 구


![image](https://github.com/user-attachments/assets/6d04d220-4504-4674-9ad3-c53b06afda92)


# Jetson (카메라 Publisher 노드)
```
⦁ GStreamer를 사용하여 Jetson 카메라 영상 입력
⦁ OpenCV를 통해 프레임 캡처
⦁ sensor_msgs::msg::CompressedImage 형식으로 변환
⦁ ROS2 토픽 /image/compressed로 Publish
```

# Jetson (카메라 Subscriber 노드)
```
⦁ GStreamer를 사용하여 Jetson 카메라 영상 
```
