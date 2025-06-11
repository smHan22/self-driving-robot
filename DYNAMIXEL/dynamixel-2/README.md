# Dynamixel-2

➨ Jetson에서 카메라 영상을 publish
➨ WSL에서 해당 영상을 수신해 영상 처리를 수행
➨ Jetson에서 바퀴를 제어할 속도를 Publish
➨ Jetson에서 이를 수신해 바퀴를 제어


![image](https://github.com/user-attachments/assets/ad843665-de34-4a08-b07d-6c3b65c92fa6)


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

# Jetson (바퀴 제어 명령 Publisher 노드)
```
⦁ 키보드 입력으로부터 DXL 바퀴의 목표 속도 결정
```

# Jetson (바퀴 제어 명령 Subscriber 노드)
```
⦁ dxlpub 토픽 수신 -> 수신한 속도로 나이나믹셀 제어
⦁ Jetson 내에서 직접 바퀴 속도 제어 수행
```
