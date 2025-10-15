## **💡1. 프로젝트 개요**

**1-1. 프로젝트 소개**
- 프로젝트 명 : 다양한 요리가 가능한 강화학습 기반 요리 로봇 개발
- 프로젝트 정의 : 어떤한 메뉴가 되던 하나의 하드웨어 장치(로봇팔)로 학습된 모델을 이용해서 어떤 요리든지 가능할 수 있도록 하는 기술개발
  
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
 강화학습을 통해 다양한 요리 데이터를 학습하여 사용자의 주문에 따라 요리 
레시피를 자동으로 수행하고, 카메라 및 센서를 통해 주변환경과 재료를 정확히 
인식하여 상황에 맞는 최적의 조리를 실행하며, 키오스크를 통해 실시간으로 요리 
주문 및 진행상황을 확인하고 관리할 수 있도록 한다.

**1-5. 기대 효과 및 활용 분야**
- 기대효과:강화학습 기반 로봇팔로 기존 로봇 대비 작업 유연성 및 효율성 증대, AI 요리 로봇팔로 외식업 인력난 해소 및 운영비용 절감
- 활용분야: 식당, 카페, 등 외식업의 조리 및 반복적 자동화로 인력 부족 해결 및 품질 유지, 제조공장 내 반복업무 효율성 및 안전성 강화-IOT, AI 기술 기반의 헬스케어, 스마트 시티 등 다양한 미래산업 응용-로봇 및 자동화 분야의 최신기술을 실무 교육 콘텐츠로 활용 가

**1-6. 기술 스택**
- 하드웨어 : 3D프린팅, openCR 1.0, Dynamixel(모터 XM430-W350), RGB 카메라
- 앱개발 : Python(FastAPI), Eleven Labs, HTML, Stitch/Figma
- 데이터수집/강화학습: Ubuntu 22.04(ROS2 Humble), Docker(ROS1 noetic), Mujoco, Groot N1, ROSbag, HuggingFace
- 배포 및 관리 : GitHub(코드), Notion(회의록), Google Dirve(문서작업)

---
## **💡2. 팀원 소개**
| <img width="80" height="100" src="https://github.com/user-attachments/assets/ab73bb1c-c1d4-464d-8ad3-635b45d5a8ae" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c7f66b7c-ab84-41fa-8fba-b49dba28b677" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c33252c7-3bf6-43cf-beaa-a9e2d9bd090b" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/0d5909f0-fc73-4ab9-be09-4d48e3e71083" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c7f66b7c-ab84-41fa-8fba-b49dba28b677" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c7f66b7c-ab84-41fa-8fba-b49dba28b677" > |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **서우영** | **손창우** | **백정훈** | **오민재** | **윤중현** | **이경용** |
| • 강화학습 <br> • 백엔드 | • 개발총괄 <br> • 강화학습 | • 하드웨어 설계 <br> • 하드웨어 제작 |• 하드웨어 제작 <br> • 데이터셋 추출 | • 앱개발 <br> • 데이터셋 추출 | • 프로젝트 멘터 <br> • 기술 자문 |




---
## **💡3. 시스템 구성도**

- **하드웨어**
<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/838d832f-f93e-4cc7-9c5d-9f962f466de3" /></br>

- **데이터 수집**
<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/61b9fbbf-70be-46db-a72f-c23986836735" /></br>

- **강화학습**
<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/3b190ffe-5a2f-4c46-981c-3d585a100384" /></br>

- **앱 서비스**
<img width="657" height="362" alt="Image" src="https://github.com/user-attachments/assets/030db7ef-6ccf-489f-b1f6-55e90b4bb68d" /></br>

---
## **💡4. 작품 소개영상**
[![Cooking Robot](https://github.com/user-attachments/assets/bf3d153b-ea08-48a6-86d5-fa1ce0a03ea0)](https://www.youtube.com/watch?v=FMgU_Z2daZs)









