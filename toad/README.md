
# 두꺼비 (Dokkaebi)

> 책상 위 작은 쓰레기 수거 및 소(큰 로봇)와 협업하는 로봇

## 시스템 구성
| 부품 | 사양 | 비고 |
|------|------|------|
| MCU | STM32F4 | 메인 제어 |
| 서브 MCU | ESP32 | 통신 제어 |
| 주행 | DC Motor x 2 | PID 제어 |
| 로봇팔 | Servo Motor x 2 | 그리퍼 포함 |
| 센서 | RaspberryPi Cam | Object Detection |
| 배터리 | 12V 리튬이온 | 독립 전원 |

## 핀맵 정보 (STM32)
| 기능 | 핀번호 | 메모 |
|------|--------|------|
| Left Motor PWM | PA0 | 좌측 모터 속도 제어 |
| Right Motor PWM | PA1 | 우측 모터 속도 제어 |
| Arm Servo1 | PB0 | 로봇팔 서보 |
| Arm Servo2 | PB1 | 그리퍼 서보 |
| ESP32 UART TX | PA9 | ESP32 연결용 |
| ESP32 UART RX | PA10 | ESP32 연결용 |
| Ultrasonic Trigger | PC0 | 장애물 감지 |
| Ultrasonic Echo | PC1 | 장애물 감지 |
