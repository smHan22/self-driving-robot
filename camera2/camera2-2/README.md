# Camera 2-2

➨ Jetson의 CSI 카메라로부터 실시간 영상을 캡처하고 ROS2를 통해 토픽으로 전송
➨ WSL에서 ROS2 토픽을 Subscribe하여 영상 수신 -> 영상 처리 및 시각화
➨ 영상 저장


![image](https://github.com/user-attachments/assets/9751efa4-6a9c-4b23-b7d0-3bf0bb578610)


# Jetson (Publisher 노드)
```
⦁ GStreamer를 이용해 Jetson CSI 카메라 영상 받음
⦁ OpenCV로 프레임 획득 및 압축 메시지 변환
⦁ ROS2 토픽 /image/compressed로 Publish
```

# WSL (Subscriber 노드)
```
⦁ /image/compressed 토픽 수신
⦁ OpenCV로 복원 및 이미지 처리
⦁ 원본 영상, 그레이영상, 이진화 영상 시각화
⦁ 영상 저장
```
