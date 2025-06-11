# Dynamixel-1

➨ WSL에서 사용자의 키보드 입력으로 다이나믹셀 바퀴 제어 명령을 생성하여 Jetson으로 전송하고, Jetson에서는 이를 수신해 바퀴 제어


![image](https://github.com/user-attachments/assets/2f0f1991-c66a-47b0-ae40-f21d930dcfb8)


# WSL (Publisher 노드)
```
⦁ 사용자가 cin으로 입력한 명령을 통해 바퀴 속도를 결정함
```

# jetson (Subscriber 노드)
```
⦁ dxl 객체를 통해 실제 Dynamixel 장치를 제어함
⦁ 수신된 메시지의 x, y값을 각각 왼쪽/오른쪽 바퀴 속도로 해석하여 dxl.setVelocity()로 설정함
⦁ 종료시에는 dxl.close()로 포트 닫음
```
