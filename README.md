
## https://youtu.be/FMgU_Z2daZs?si=NH6PZwmL2qrZCTWO
<p align="center">
  <img src="https://github.com/user-attachments/assets/2bb29113-8727-4c94-9818-f9b232b0c370" width="700" height="auto">
</p>

# DataCollection
<p align="center">
  <img src="https://github.com/user-attachments/assets/16122e49-710c-4e9e-ad2c-60a0b3e6f54e" width="700" height="auto">
</p>


리더, 팔로우 로봇 캘리브레이션 후, record.sh 파일 실행하여 데이터셋 수집

이때, ls /dev/ttyACM* 명령어를 실행하여 USB 시리얼 포트를 확인
## 캘리브레이션
calibrate.sh 파일 실행하여 캘리브레이션

로봇의 그리퍼를 벌리고, 로봇팔을 최대한 'ㄱ'자로 만든 뒤에 진행할 것

이후, 모터의 최대 구동 범위까지 움직여줄 것
### 팔로워 로봇 캘리브레이션 
```
lerobot-calibrate \
    --robot.type=bi_so101_follower \
    --robot.left_arm_port=/dev/ttyACM0 \    
    --robot.right_arm_port=/dev/ttyACM1 \
    --robot.id=dual_so101
```
### 리더 로봇 캘리브레이션
```
lerobot-calibrate \
    --teleop.type=bi_so101_leader \
    --teleop.left_arm_port=/dev/ttyACM2 \
    --teleop.right_arm_port=/dev/ttyACM3 \
    --teleop.id=dual_leader_so101
```
## 하드웨어 구성
### 팔로워 로봇 (bi_so101_follower)
왼팔: /dev/ttyACM0

오른팔: /dev/ttyACM1

설정 위치: lerobot/common/robot_devices/robots/configs.py - BiSO101FollowerConfig 클래스
### 리더 로봇 (bi_so101_leader)
왼팔: /dev/ttyACM2

오른팔: /dev/ttyACM3

설정 위치: lerobot/common/robot_devices/robots/configs.py - BiSO101LeaderConfig 클래스
### 카메라: 4대의 OpenCV 카메라
카메라 인덱스는 find_cameras.py 코드를 통해 확인

정면 카메라: index 0

상단 카메라: index 2

왼쪽 카메라: index 4

오른쪽 카메라: index 6

## 데이터셋 저장
로컬: --dataset.root에 지정한 경로

클라우드: --dataset.repo_id로 Hugging Face Hub에 업로드
# TrainAndDelpoy
로봇에 배포시 두개의 터미널을 사용한다.
한개는 서버실행, 한개는 sh파일 실행에 쓰인다
## 서버 실행
```
python scripts/inference_service.py \
    --server \
    --http-server \
    --model-path {체크포인트 경로} \
    --embodiment-tag new_embodiment \
    --data-config so101
```
새로운 터미널을 열어서 아래코드 실행한다
```
eval.sh
```

터미널에서 위 명령어를 실행하여 모델 서버 실행


<details>
    <summary>각 파라미터 설명</summary>

<!-- summary 아래 한칸 공백 두고 내용 삽입 -->


 - **python scripts/inference_service.py**
  
  추론 서비스 스크립트 실행
  
  학습된 모델을 로드하고 서버로 실행
  
  
  - **--server**
  
  서버 모드로 실행
  
  로봇의 요청을 받아 AI 추론 결과를 반환
  
  
  - **--http-server**
  
  HTTP 프로토콜 사용
  
  REST API 형태로 통신 가능
  
  로봇이 HTTP 요청으로 다음 동작을 물어봄
  
  
  - **--model-path {모델 경로}**
  
  학습된 모델의 경로 지정
  
  
  - **--embodiment-tag new-embodiment**
  
  로봇의 물리적 구성(embodiment) 태그
  
  같은 모델을 다른 로봇 구성에 적용할 때 사용
  
  new-embodiment: 새로운/커스텀 로봇 설정
  
  
  - **--data-config so101**
  
  데이터 구성 설정
  
  so101: SO-101 로봇용 데이터 형식
  
  입력 데이터(카메라, 관절 상태 등)의 구조 정의
  
</details>

## 로봇 실행
데이터셋 수집과 마찬가지로 USB 포트 번호를 통해 팔로워 로봇 설정해준 뒤, 카메라 인덱스 설정


**--policy_host=localhost & --policy_port=5555** 를 통해 AI 모델이 실행 중인 서버 위치를 지정

이후 localhost:5555에서 실행 중인 inference 서버와 연결하면 이 서버가 로봇에게 다음에 어떻게 움직여야 할지 알려줌


**--lang_instruction** 을 통해 AI에게 주는 작업 지시

GR00T N1의 Vision-Language Model이 이 지시를 이해하고 실행
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
<p align="center">
  <img src="MetaQuest_Teleopearation/assets/IMG_1756.gif" width="700" height="auto" alt="Single Arm Demo">
</p>


*Meta Quest 2 VR 컨트롤러로 한팔 OpenManipulator-X 로봇을 실시간 제어*

### Dual Arm VR 제어
![Dual Arm Demo](MetaQuest_Teleopearation/assets/IMG_1862.gif)

*양팔 로봇 동시 제어 *


## 주요 성과
- Single Arm 및 Dual Arm VR 제어 구현
- Offset-based Control 알고리즘 개발
- Wifi 도시락을 사용해 장소 제한 없이 구현

## 디렉토리 구조 및 설명

<img width="2375" height="1450" alt="image" src="https://github.com/user-attachments/assets/06ed5365-9f08-4b98-aa1a-f4a0b51f1b89" />



<details>
    <summary>MetaQuest_Teleopearation 파일구조조</summary>

<!-- summary 아래 한칸 공백 두고 내용 삽입 -->

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

  
</details>

## 상세 설명 (링크 클릭하시면 상세한 설명이 있습니다.)
- [VR 시스템 상세](MetaQuest_Teleopearation/vr_teleoperation/README.md)
- [하드웨어 설정](MetaQuest_Teleopearation/open_manipulator/HARDWARE_SETUP_GUIDE.md)
- [Docker 설정](MetaQuest_Teleopearation/docker/docker-setup-guide.md)
- [데이터 수집](MetaQuest_Teleopearation/data/README.md)



## 기술 스택
- **VR**: Meta Quest 2, quest2ros
- **로봇**: OpenManipulator-X (Dynamixel XM430-W350)
- **미들웨어**: ROS2 Humble, ROS1 Noetic (Docker)
- **시뮬레이션**: MuJoCo 2.3+
- **언어**: Python 3.10+
- **OS**: Ubuntu 22.04






































