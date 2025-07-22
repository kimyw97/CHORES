
# 소 (Cow)

> 바닥 쓰레기 수거 및 두꺼비(작은 로봇) 지원 로봇

## 임베디드
- stm32폴더는 stm32f4discovery 기반으로 작성된 코드입니다. stm32 프로젝트이니 ioc 파일 기준으로 새로 만들어서 하는 것을 추천드립니다.
- stm32_v2 폴더는 stm32f3discovery 기반으로 작성된 코드입니다.(개인용은 f3여서 바꿈)
- 주로 제어와 센서리딩 같은 로우 레벨에서의 작업만 해두었습니다.

## SLAM, Nav2
1. raspberrypi
   라즈베리파이에서 소스코드 복제 후 빌드를 먼저 해줍니다. 물론 ros 관련 세팅이 다 되어 있다는 전제하에 그렇습니다.
   ```bash
   colcon build --packages-select cow_odom_publisher robot_monitoring
   ```
2. 노트북(ubuntu 22.04)
   노트북에서는 이제 모든 소스코드를 빌드해줍니다.
   ``` bash
   colcon build
   ```
   간혹 성능이 안좋은 노트북의 경우 멈출 수 있으니 parallel workers 옵션을 사용하는 걸 추천드립니다.

3. slam 명령어(순서 중요!)
   - stm32로부터 정보를 받는 라즈베리 파이로 가서 아래 명령어
   ``` bash
   ros2 launch cow_odom_publisher cow.launch.py
   ```
   - 아래 명령어는 같은 네트워크 상이면 아무곳이든 상관없음
   ```bash
    ros2 launch robot_monitoring robot_description.launch.py #tf pub
    ros2 launch cow_odom_publisher ekf_localization.launch.py #ekf
    ros2 launch nav2_bringup slam_launch.py slam_params_file:=/home/kimyw/vscode/CHORES/cow/raspberrypi/ros2_ws/src/navigation2_custom/nav2_bringup/params/slam_config.yaml # slam, 경로는 사용자에 맞게 변경
    ```
4. nav2 명령어(순서 중요!)
   - stm32로부터 정보를 받는 라즈베리 파이로 가서 아래 명령어
   ``` bash
   ros2 launch cow_odom_publisher cow.launch.py
   ```
   - 아래 명령어는 같은 네트워크 상이면 아무곳이든 상관없음
   ```bash
    ros2 launch robot_monitoring robot_description.launch.py #tf pub
    ros2 launch cow_odom_publisher ekf_localization.launch.py #ekf
    ros2 launch nav2_bringup localization_launch.py map:="/home/kimyw/vscode/CHORES/cow/raspberrypi/ros2_ws/src/navigation2_custom/nav2_bringup/map/test_map.yaml" use_sim_time:=false # amcl
   ros2 launch nav2_bringup bringup_launch.py map:="/home/kimyw/vscode/CHORES/cow/raspberrypi/ros2_ws/src/navigation2_custom/nav2_bringup/map/test_map.yaml" use_sim_time:=false # nav2

   ```
  
  
