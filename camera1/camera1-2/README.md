# 섭스크라이버 노드에서 구독한 영상을 동영상 파일(mp4)로 저장하는 패키지

➨ Jetson 내부에서 카메라 영상을 ROS2를 통해 Publish한 뒤, Jetson 내 Subscriber 노드에서 해당 영상을 수신하고 OpenCV로 이진화 처리 후 외부 PC로 GStreamer 스트리밍 전송
➨ Jetson 내부에 영상 저장

![image](https://github.com/user-attachments/assets/c8f3b5c0-33c5-4c9f-ba23-d6fdeeddaae5)

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
⦁ 동시에 output.mp4 파일로 저장
```

# PC (영상 확인)
```
⦁ Jetson에서 전송한 UDP 영상 수신
⦁ GStreamer 기반 수신 파이프라인을 사용해 영상 확인 가능
```
