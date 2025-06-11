# Lidar-2

![image](https://github.com/user-attachments/assets/52f5475b-3f4a-4ff8-915a-9c8a9af8002f)


# Rplidar C1의 좌표축을 설명하라.
```
⦁ 라이다 센서의 앞쪽을 가리키는 x축, 좌우 방향의 y축
```

# Rplidar C1은 1초에 몇 번 토픽메시지를 전송하는가?
```
⦁ 초당 약 10번 정도 토픽 메시지를 전송, 데이터 전송 주기는 대략 100밀리초 (0.1초) 간격
⦁ ros2 topic lsit
⦁ ros2 topic bw /scan
```

# 토픽메시지의 크기(KB)는 얼마인가?
```
⦁ 대략 메시지 하나당 5.82KB이므로 10번 전송된다고 하면 초당 58.88KB의 크기로 전송
⦁ ros2 topic list
⦁ ros2 topic bw /scan
```

# 메시지 1개당 몇 개의 거리 측정값이 포함되어 있는가?, 1회전에 몇 번 거리를 측정하는가?
```
⦁ ranges 배열의 항목 수 확인
⦁ ros2 topic echo /scan
⦁ /scan 토픽에서 angle_increment 값을 찾으면 됨
⦁ angle_increment 값은 라디안 단위로 저장됨
⦁ 한 회전 360도 = 2파이 라디안 기준으로 특정 포인트 수는 2 x 3.141592 / angle_increment 값
```

# angle_min, angle_increment 값은 얼마인가?
```
⦁ ros2 topic echo /scan 내용에서 확인 가
```
