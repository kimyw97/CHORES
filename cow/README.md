
# 소 (Cow)

> 바닥 쓰레기 수거 및 두꺼비(작은 로봇) 지원 로봇

## 시스템 구성
| 부품 | 사양 | 비고 |
|------|------|------|
| MCU | STM32F4 | 메인 제어 |
| 서브 MCU | ESP32 | 통신 제어 |
| 주행 | DC Motor x 2 | PID 제어 |
| 센서 | LiDAR | SLAM 지도 생성 |
| 리프트 | Linear Actuator | 두꺼비 승강 |
| 쓰레받기 | Servo Motor | 쓰레기 수거 구조 |
| 배터리 | 24V 리튬이온 | 고출력 |

## 핀맵 정보 (STM32)
| 기능 | 핀번호 | 메모 |
|------|--------|------|
| Left Motor PWM | PA0 | 좌측 모터 속도 제어 |
| Right Motor PWM | PA1 | 우측 모터 속도 제어 |
| Lift Motor PWM | PB0 | 리니어 액추에이터 제어 |
| Dustpan Servo | PB1 | 쓰레받기 개폐 |
| ESP32 UART TX | PA9 | ESP32 연결용 |
| ESP32 UART RX | PA10 | ESP32 연결용 |
| LiDAR UART TX | PC4 | LiDAR 연결 |
| LiDAR UART RX | PC5 | LiDAR 연결 |
