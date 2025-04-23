시스템 토픽 흐름 (rqt_graph)

rosgraph.png은 rqt_graph를 통해 시각화한 RPLIDAR + Cartographer SLAM 노드와 토픽 흐름도입니다. 주요 구성은 다음과 같습니다:

1.주요 노드
노드 이름	설명
/rplidar_node	RPLIDAR에서 실제 센서 데이터를 수집하여 /scan 토픽으로 퍼블리시
/cartographer_node	/scan 토픽을 수신하여 SLAM 알고리즘을 통해 지도와 위치를 추정
/occupancy_grid_node	Cartographer의 서브맵 정보를 받아 /map 토픽으로 occupancy grid 생성
/static_transform_publisher_*	정적인 좌표계 변환 (base_link ↔ laser, 등)을 제공
/transform_listener_impl_*	TF 프레임 전파 및 수신 처리

2.토픽 흐름

    /rplidar_node가 /scan 토픽으로 라이다 스캔 데이터를 발행합니다.

    /cartographer_node는 이 /scan 데이터를 수신하여 SLAM을 수행하고, 서브맵 정보를 /submap_list로 발행합니다.

    /occupancy_grid_node는 /submap_list를 바탕으로 /map 토픽을 생성하여 RViz나 다른 노드에서 지도 시각화를 가능하게 합니다.

    여러 static_transform_publisher_* 노드가 레이저 프레임과 로봇 기본 프레임 (base_link) 간의 고정된 좌표 변환을 제공합니다.

3. 실행 방법

1. RPLIDAR 드라이버 실행
cd ros2_ws
ros2 launch rplidar_ros rplidar.launch.py

기본 포트 /dev/ttyUSB0 사용, 필요시 launch 파일 수정

2. Cartographer SLAM 실행
cd ros2_ws
ros2 launch jdamr200_cartographer cartographer.launch.py


3. RViz 시각화 실행

/scan, /map, /tf 프레임을 기반으로 실시간 맵 시각화 가능

4. rqt_graph 토픽 흐름 확인

rqt_graph