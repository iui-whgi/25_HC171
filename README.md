## **💡1. 프로젝트 개요**

**1-1. 프로젝트 소개**
- 프로젝트 명 : 다양한 요리가 가능한 강화학습 기반 요리 로봇 개발
- 프로젝트 정의 : 어떤한 메뉴가 되던 하나의 하드웨어 장치(로봇팔)로 학습된 모델을 이용해서 어떤 요리든지 가능할 수 있도록 하는 기술개발</br>  
  <img width="835" height="460" alt="Image" src="https://github.com/user-attachments/assets/4472a4c9-1271-4526-bde4-07cba5d5a504" /></br>

**1-2. 개발 배경 및 필요성**
- 최근 외식업 산업 및 생산 분야의 업무 강도가 높아 인력확보에 어려움을 겪고 있다. 특히, 1인가구 및 맞벌이 가정 증가로 인해 외식 수요가 늘어남에 따라 주방일의 업무 강도는 더 높아졌기에 직원 구하기가 더욱 어려운 상황이다. 이러한 인력 문제를 로봇을 이용하여 해결하기 위한 노력이 점차 증가하고 있다. 그 중 로봇팔을 기반으로 한 자동화 산업의 수요가 증가하고 있어 주로 공장 등 자동화 산업 시설에 주로 도입되고 있다. 또한, 외식업 주방 및 거피 바리스타 등에도 점차 도입되고 있는 추세이다. 하지만, 요리 및 메뉴에 따라 제한적으로만 사용되고 있어 추가적인 도입 비용이 발생하며, 매번 추가 개발이 필요하게 된다.

**1-3. 프로젝트 특장점**
- 유연성/적응력: 환경 변화·새 메뉴 추가 시 데이터만으로 자동 학습 가능.
- 확장성: 단일 하드웨어로 재료 손질→조리→플레이팅 등 복합 작업 자동화.
- 비용 효율성: 데이터셋 추가만으로 새로운 작업 적용, 개발·운영비 절감.
- 실시간 최적화: 센서 기반 환경 인식으로, 조리경로 산출 및 품질 일관성 확보.
- 사용자 편의: 키오스크 연동 레시피 선택·진행 모니터링 지원.

**1-4. 주요 기능**
- 강화학습을 통해 다양한 요리 데이터를 학습하여 사용자의 주문에 따라 요리 
레시피를 자동으로 수행하고, 카메라 및 센서를 통해 주변환경과 재료를 정확히 
인식하여 상황에 맞는 최적의 조리를 실행하며, 키오스크를 통해 실시간으로 요리 
주문 및 진행상황을 확인하고 관리할 수 있도록 한다.

**1-5. 기대 효과 및 활용 분야**
- 기대효과: 강화학습 기반 로봇팔로 기존 로봇 대비 작업 유연성 및 효율성 증대, AI 요리 로봇팔로 외식업 인력난 해소 및 운영비용 절감
- 활용분야: 식당, 카페, 등 외식업의 조리 및 반복적 자동화로 인력 부족 해결 및 품질 유지, 제조공장 내 반복업무 효율성 및 안전성 강화-IOT, AI 기술 기반의 헬스케어, 스마트 시티 등 다양한 미래산업 응용-로봇 및 자동화 분야의 최신기술을 실무 교육 콘텐츠로 활용 가

**1-6. 기술 스택**
- 하드웨어 : 3D프린팅, openCR 1.0, Dynamixel(모터 XM430-W350), RGB 카메라
- 앱개발 : Python(FastAPI), Eleven Labs, HTML, Stitch/Figma
- 데이터수집/강화학습: Ubuntu 22.04(ROS2 Humble), Docker(ROS1 noetic), Mujoco, Groot N1, ROSbag
- 배포 및 관리 : GitHub(코드), Notion(회의록), Google Dirve(문서작업), HuggingFace(데이터셋)

---
## **💡2. 팀원 소개**
| <img width="80" height="100" src="https://github.com/user-attachments/assets/f3dbd480-b7f3-49d7-aaa4-e929b7cc384b" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/2947cd40-49dc-4845-b602-cebe30b9c98a" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/6201ef25-f1a2-49ce-8442-ce85e15a990a" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c617af9a-110d-4860-accc-49c9206691d7" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/a9968bc1-31b0-44b6-8b81-ef8681e35488" > | <img width="80" height="100" alt="멘토" src="https://github.com/user-attachments/assets/6e89de64-6438-4568-a745-1d404ef9c6f0" > |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **서우영** | **손창우** | **백정훈** | **오민재** | **윤중현** | **이경용** |
| • 데이터셋 수집 <br> • 백엔드 | • 개발총괄 <br> • 강화학습 | • 하드웨어 설계 <br> • 하드웨어 제작 |• 하드웨어 제작 <br> • 데이터셋 추출 | • 프론트엔드 <br> • 데이터셋 추출 | • 프로젝트 멘토 <br> • 기술 자문 |

---
## **💡3. 시스템 구성도**

- **하드웨어**</br>

<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/838d832f-f93e-4cc7-9c5d-9f962f466de3" /></br>

- **데이터 수집**</br>

<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/61b9fbbf-70be-46db-a72f-c23986836735" /></br>

- **강화학습**</br>

<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/3b190ffe-5a2f-4c46-981c-3d585a100384" /></br>

- **웹 페이지**</br>

<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/030db7ef-6ccf-489f-b1f6-55e90b4bb68d" /></br>

---
## **💡4. 작품 소개영상**
[![Cooking Robot](https://github.com/user-attachments/assets/bf3d153b-ea08-48a6-86d5-fa1ce0a03ea0)](https://www.youtube.com/watch?v=FMgU_Z2daZs)

---
## **💡5. 핵심 소스코드**
- 가상환경 내에 로봇팔 정의(왼쪽 기준)</br>

<img width="855" height="408" alt="Image" src="https://github.com/user-attachments/assets/396471b9-c36d-4287-982b-703c81be8281" /></br>

- 엑츄에이터 정의</br>
<img width="900" height="142" alt="액추에이터" src="https://github.com/user-attachments/assets/b88b5ef9-c540-43db-817b-0d201fa2c074" /></br>

- .act 파일을 읽어 재생 시간 데이터와 액추에이터 제어 입력 시퀀스를 로드</br>
<img width="497" height="432" alt="act파일읽어시퀌스뭐시기저시기" src="https://github.com/user-attachments/assets/d67b19d5-8b95-4f7d-85e4-9bc4f058a425" /></br>

- 보간한 제어 입력을 Mujoco 액추에이터에 적용해 시뮬레이션을 한 단계 진행</br>
<img width="441" height="250" alt="보간한 제어입력" src="https://github.com/user-attachments/assets/27e1db5b-d295-425b-b9c7-5d405cecba02" /></br>

- Bag 파일로부터 관절 위치 데이터 로드</br>
<img width="390" height="492" alt="bag파일로부터 관절위치데이터로드" src="https://github.com/user-attachments/assets/f04e63ac-e802-4a5e-83ec-abbdddf44ad9" /></br>

- actuator mapping & .act 변환</br>
<img width="855" height="408" alt="Image" src="https://github.com/user-attachments/assets/c5252405-4d2f-47e8-88dc-c96cd3cca28c" /></br>

- VR-joint 매핑 데이터 구축</br>
<img width="482" height="557" alt="VR-joint매핑" src="https://github.com/user-attachments/assets/3f564fbd-1291-4ad5-96ab-dafa80121ff7" /></br>

- KD-tree 구성 및 VR 델타값 계산</br>
<img width="491" height="560" alt="KDtree구성 vr계산" src="https://github.com/user-attachments/assets/3acaa775-3f1a-46a3-b531-7016b01bf701" /></br>

- KD-tree 알고리즘</br>
<img width="476" height="560" alt="kd알고리즘" src="https://github.com/user-attachments/assets/63b732de-2e8b-4197-a136-66d1ded50313" />

<img width="657" height="561" alt="image" src="https://github.com/user-attachments/assets/ffa95885-1180-4620-9740-d5f1d6dbe6e4" /></br>

- 소켓 서버 설정(Mujoco 연결 대기)</br>
<img width="487" height="283" alt="소켓서버설정" src="https://github.com/user-attachments/assets/e4f4dd2a-bb79-4bcd-bf9c-cf52f1a4f07a" /></br>

- ROS 토픽구독(VR데이터 입력)</br>
<img width="485" height="257" alt="ROS토픽구독" src="https://github.com/user-attachments/assets/dc6381f5-6b35-423c-ab66-853998373600" /></br>

- Mujoco로 데이터 전송 및 제어루프</br>
<img width="546" height="601" alt="mujoco데이터전송" src="https://github.com/user-attachments/assets/65ab43f1-f9b4-4ebf-a543-4d6463f0bd5c" /></br>

- 실제 로봇과 Mujoco 환경을 동기화</br>
<img width="546" height="601" alt="실제로봇mujoco환경동기화" src="https://github.com/user-attachments/assets/d1720111-fab3-4690-87b7-c0bf8a6ec2f2" /></br>

- 실제 로봇의 초기위치 값 기억</br>
<img width="440" height="247" alt="실제로봇 초기위치" src="https://github.com/user-attachments/assets/93987adc-7af3-431a-b01b-08953b3aafc9" /></br>

- 초기 위치 기준으로 변화량만 확인 후, 로봇팔 동작</br>
<img width="657" height="565" alt="zzzz" src="https://github.com/user-attachments/assets/3de4a639-9027-4904-8ec0-2d39523643a6" /></br>





