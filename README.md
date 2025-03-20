YOLO Depth ROS
YOLO Depth ROS는 객체 감지와 깊이 추정(Depth Estimation)을 결합하여 3D 공간상의 객체 위치를 추정할 수 있도록 하는 ROS2 패키지입니다.

구성
yolo_depth_bringup
런치 파일과 관련 설정을 제공합니다.

yolo_depth_ros
YOLO 모델과 깊이 추정 모델을 활용하여 메시지를 발행/구독하고 처리하는 ROS2 노드 구현부입니다.

yolo_msgs
객체 감지와 깊이 정보에 대한 커스텀 메시지 타입을 정의합니다.

특징
객체 감지: Ultralytics YOLO 모델을 통해 이미지에서 객체를 감지합니다.
깊이 추정: 미리 학습된 Depth Estimation 모델을 사용하여 감지된 객체까지의 깊이를 추정합니다.
3D 좌표 계산: 카메라 내부 파라미터와 깊이 정보를 조합해 객체의 (X, Y, Z) 좌표를 산출합니다.
RViz 시각화: 결과를 이미지 토픽, MarkerArray 등을 통해 시각화할 수 있습니다.

설치 및 빌드
ROS2 (Humble 버전 등) 환경을 준비합니다.

``` pip install requirements.txt ```

빌드:
``` colcon build --symlink-install ```
환경 설정:
``` docker build -t yolo_depth_ros . ```
실행 방법
아래 명령어를 통해 YOLO Depth ROS 노드를 실행할 수 있습니다:

``` ros2 launch yolo_depth_bringup yolo_depth.launch.py det_model:=best_mm.pt input_image_topic:=image_raw use_tracking:=True ```

``` docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix yolo_depth_ros:latest ```


