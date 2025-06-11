# Camera 1-1

➨ 본 시스템은 Jetson에서 실시간으로 영상을 받고, GStreamer를 통해 PC로 전송하는 도구


![image](https://github.com/user-attachments/assets/6d04d220-4504-4674-9ad3-c53b06afda92)


# Jetson (카메라 Publisher 노드)
```
⦁ GStreamer를 통해 카메라 영상 읽기
⦁ OpenCV로 프레임 처리
⦁ sensor_msgs::msg::CompressedImage 형식으로 변환
⦁ ROS2 토픽 image/Compressed로 Publish
```

# Jetson (Subscriber 영상 전송 노드)
```
⦁ ROS2 토픽 image/compressed 수신
⦁ OpenCV로 디코딩 및 이진화
⦁ GStreamer를 통해 PC로 UDP 영상 스트리밍 전송
```

# PC (영상 확인)
```
⦁ Jetson에서 전송한 UDP 영상 수신
⦁ GStreamer 기반 수신 파이프라인을 사용해 영상 확인 가능
```
