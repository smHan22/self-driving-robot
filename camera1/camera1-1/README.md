# Jetson 카메라 영상을 PC로 전송하여 출력하는 패키지

➨ 본 패키지는 Jetson에서 촬영한 카메라 영상을 ROS2를 이용해 PC로 실시간 전송하고, 수신된 영상을 이진화 처리 후 CStreamer 파이프라인을 통해 영상을 확인하는 구조로 구성되어 있습니다.


![image](https://github.com/user-attachments/assets/6d04d220-4504-4674-9ad3-c53b06afda92)

# Jetson (Publisher 노드)
```
⦁ 역할: 카메라 영상 캡처 및 ROS2 토픽으로 압축 이미지 전송
⦁ GStreamer를 통해 카메라로부터 영상을 받음
⦁ OpenCV를 이용하여 프레임을 sensor_msgs::msg::CompressedImage 형식으로 압축
⦁ ROS2 토픽 image/compressed
```

# PC (Subscriber 노드)
```
⦁ 역할: 압축 이미지 수신 -> 디코딩 -> 이진화 처리 -> 영상 확인
⦁ image/compressed 토픽 수신
⦁ 수신한 압축 이미지를 그레이스케일로 변환
⦁ 임계값 100기준으로 이진화
⦁ 이진화 이미지를 BGR 포맷으로 복원 후 영상 출
```
