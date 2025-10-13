# CookingBot


각자 맡은 파트 자세히 명세해주세요.

DataCollection (지수씨) - Isaac ~Bimanaul  파일넣어주세요
TrainAndDelpoy (지수씨) - Gr00t 디렉토리 넣어주세요 
MetaQuest_Teleopearation(숭실대팀) - 텔레오퍼레이션했던거 다정리해주세요

각자 명세하신후 우영이가 지수씨 파트를 더 보완하고 , 종합적으로는 숭실대 팀이 맡아서 관리좀해주세요. 



#이 Read Me에 각 디렉별 설명 다 기입해주시면됩니다 . 다 이 제일상위 Readme에 다 명세해주세요

# DataCollection
직접 작성한 record.sh 파일 실행하여 데이터셋 수집

## 하드웨어 구성
### Follower 로봇 (bi_so101_follower)
왼팔: /dev/ttyACM0

오른팔: /dev/ttyACM1

설정 위치: lerobot/common/robot_devices/robots/configs.py - BiSO101FollowerConfig 클래스
### Leader 로봇 (bi_so101_leader)
왼팔: /dev/ttyACM2

오른팔: /dev/ttyACM3

설정 위치: lerobot/common/robot_devices/robots/configs.py - BiSO101LeaderConfig 클래스
### 카메라: 4대의 OpenCV 카메라
정면 카메라: index 0

상단 카메라: index 2

왼쪽 카메라: index 4

오른쪽 카메라: index 6

# TrainAndDelpoy

---

# MetaQuest_Teleopearation

## 개요
Meta Quest 2 VR 헤드셋을 활용하여 OpenManipulator-X 로봇을 직관적으로 제어하는 텔레오퍼레이션 시스템입니다.

기존의 복잡한 역기구학(IK) 대신 **Offset-based Control** 방식을 도입하여 안전하고 직관적인 로봇 제어를 구현했습니다.

## 시스템 아키텍처
```
Meta Quest 2 (VR) → Docker (ROS1 + quest2ros) → Host (ROS2) → Physical Robot
                                  ↓
                             MuJoCo Simulation
```

### 통신 구조
```
Meta Quest 2 → [USB] → Docker(ROS1) → [TCP:12345] → MuJoCo
                                    ↘
                                     [TCP:12346] → ROS2 → Robot
```

## 시연 영상

### Single Arm VR 제어
![Single Arm Demo](MetaQuest_Teleopearation/assets/IMG_1756.gif)

*Meta Quest 2 VR 컨트롤러로 한팔 OpenManipulator-X 로봇을 실시간 제어*

### Dual Arm VR 제어
![Dual Arm Demo](MetaQuest_Teleopearation/assets/IMG_1862.gif)

*양팔 로봇 동시 제어 *


## 주요 성과
- Single Arm 및 Dual Arm VR 제어 구현
- Offset-based Control 알고리즘 개발
- Wifi 도시락을 사용해 장소 제한 없이 구현

## 디렉토리 구조 및 설명

### `vr_teleoperation/`
VR 텔레오퍼레이션 메인 시스템

- **`single_arm/`**: 싱글암 제어 시스템
  - `bridge_single.py`: VR 컨트롤러 데이터를 ROS1에서 수신하여 Host로 전송
  - `mujoco_single.py`: MuJoCo 시뮬레이션 환경
  - `mirror_single.py`: 실제 로봇 제어 (ROS2)

- **`dual_arm/`**: 듀얼암 제어 시스템
  - `bridge_dual.py`: 양팔 VR 데이터 브릿지
  - `mujoco_dual.py`: 듀얼암 시뮬레이션
  - `mirror_dual.py`: 양팔 로봇 동시 제어

- **`colcon_ws/`**: ROS2 워크스페이스
- **`DynamixelSDK/`**: Dynamixel 모터 제어 SDK

### `open_manipulator/`
OpenManipulator-X ROS2 하드웨어 패키지

- **`open_manipulator_x_description/`**: 로봇 URDF 모델 및 메시 파일
- **`open_manipulator_x_bringup/`**: 로봇 실행을 위한 런치 파일
- **`dynamixel_hardware_interface/`**: Dynamixel 모터 하드웨어 인터페이스
- **`dynamixel_interfaces/`**: 커스텀 ROS2 메시지 및 서비스 정의

### `docker/`
quest2ros Docker 환경 설정

- `Dockerfile`: quest2ros ROS1 컨테이너 이미지
- `docker-compose.yml`: Docker 실행 설정
- `docker-setup-guide.md`: Docker 환경 구축 가이드

### `data/`
VR-로봇 매핑 데이터 수집 도구

- `teaching.py`: 실물 로봇 Direct Teaching (토크 OFF 상태로 수동 조작)
- `recorder.py`: VR 포즈와 로봇 조인트 동시 기록
- `vr_pose.py`: VR 컨트롤러 데이터 수집기 (Docker 내 실행)
- `mujoco_data.py`: 실물 로봇과 동기화된 시뮬레이션 환경
- **`xml/`**: MuJoCo 시뮬레이션용 XML 파일

### `examples/`
예제 및 테스트 코드

- **`openmanipulator_examples/`**: OpenManipulator-X 기본 예제
- **`aloha_examples/`**: ALOHA 로봇 관련 예제

## 기술 스택
- **VR**: Meta Quest 2, quest2ros
- **로봇**: OpenManipulator-X (Dynamixel XM430-W350)
- **미들웨어**: ROS2 Humble, ROS1 Noetic (Docker)
- **시뮬레이션**: MuJoCo 2.3+
- **언어**: Python 3.10+
- **OS**: Ubuntu 22.04

## 상세 설명 (링크 클릭하시면 상세한 설명이 있습니다.)
- [VR 시스템 상세](MetaQuest_Teleopearation/vr_teleoperation/README.md)
- [하드웨어 설정](MetaQuest_Teleopearation/open_manipulator/HARDWARE_SETUP_GUIDE.md)
- [Docker 설정](MetaQuest_Teleopearation/docker/docker-setup-guide.md)
- [데이터 수집](MetaQuest_Teleopearation/data/README.md)








