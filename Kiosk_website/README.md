# 🥪 샌드위치 키오스크 시스템

**단일 Flask 서버**로 키오스크와 주문 관리를 모두 처리하는 시스템.

## 🚀 시스템 구성

### 통합 Flask 서버 (포트 6027)
- **메인 키오스크**: http://localhost:6027
- **버튼 주문**: http://localhost:6027/button
- **음성 주문**: http://localhost:6027/voice
- **주문 내역**: http://localhost:6027/data
- **API 엔드포인트**: http://localhost:6027/api/order

포트포워딩 해서 외부 IP 써도 됨. 단, chrome에서 실행했을 때 보안을 위해 마이크 허용이 막혀 있는데 이를 해결하기 위해 [chrome://flags/#unsafely-treat-insecure-origin-as-secure](chrome://flags/#unsafely-treat-insecure-origin-as-secure)에서 해당 `<External IP>:<port>`를 추가해주면 된다.

## 📋 사용 방법

### 1. 환경 설정 (선택사항)
프로젝트에서 IP 주소, 포트 번호, Agent ID 등을 환경 변수로 관리할 수 있다.
```bash
# .env 파일 생성
cp .env

# .env 파일 편집하여 실제 값 설정
nano .env
```

`.env` 파일 예시:
```env
# 서버 설정
SERVER_HOST=0.0.0.0
SERVER_PORT=<port>
SERVER_IP=<Server IP>
DEBUG=True

# Agent 설정 (ElevenLabs)
AGENT_ID=<your agent id>
```

### 2. 서버 실행
```bash
# 필요한 패키지 설치
pip install -r requirements.txt

# 통합 키오스크 서버 시작
python3 kiosk_server.py
```

서버가 실행되면:
- 🏠 **메인 키오스크**: http://localhost:6027
- 📊 **주문 내역 보기**: http://localhost:6027/data
- 🔗 **API 엔드포인트**: http://localhost:6027/api/order
- 🗑️ **데이터 초기화**: http://localhost:6027/api/clear

### 2. 키오스크 사용
1. 웹 브라우저에서 http://localhost:6027 접속
2. 주문 방법 선택:
   - **음성으로 주문하기**: 간단한 음성 주문 시뮬레이션
   - **버튼으로 주문하기**: 체크박스로 재료 선택
3. 주문 완료 후 자동으로 주문 내역 페이지가 열림

### 3. 주문 내역 확인
- 실시간으로 주문 현황 확인
- 총 주문 수, 매출, 오늘 주문 수 통계
- 5초마다 자동 새로고침
```bash
python3 order_executor.py
```
이 부분은 아직 개발 중이고, 일단 주문 내용을 바탕으로 명령어를 자동 생성한다.

## 📁 파일 구조
```
kiosk/
├── kiosk_server.py        # 통합 키오스크 서버 (메인)
├── requirements.txt       # Python 패키지 의존성
├── .gitignore            # Git 무시 파일 목록
├── .env                  # 환경 변수 설정 예시
├── main.html             # 메인 키오스크 페이지
├── button_order.html     # 버튼 주문 페이지
├── voice_order.html      # 음성 주문 페이지
├── complete.html         # 주문 완료 페이지
├── order_executor.py     # 주문 실행기 (별도)
└── README.md             # 이 파일
```

## 🎯 주요 기능

### 키오스크 기능
- ✅ 음성 주문 (시뮬레이션)
- ✅ 버튼 주문 (체크박스 선택)
- ✅ 실시간 주문 처리
- ✅ 주문 내역 자동 표시

### 관리 기능
- ✅ 실시간 주문 현황 대시보드
- ✅ 주문 통계 (총 주문 수, 매출, 오늘 주문)
- ✅ 주문 방법별 구분 (음성/버튼)
- ✅ 자동 새로고침

## 🔧 개발자 정보

### API 사용법
```javascript
// 주문 데이터 전송
const orderData = {
    method: 'button', // 또는 'voice'
    items: [
        { name: '화이트 빵', price: 3000 },
        { name: '양상추', price: 500 }
    ],
    total_price: 3500
};

fetch('http://localhost:6027/api/order', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(orderData)
});
```

### 서버 로그
서버 실행 시 콘솔에서 실시간 주문 로그를 확인할 수 있다:
```
🥪 키오스크 통합 서버가 시작됩니다...
📍 메인 키오스크: http://localhost:6027
📍 주문 내역 보기: http://localhost:6027/data
📦 새 주문 접수: ORD1234567890
   주문 방법: 버튼 주문
   주문 시간: 2025-01-16 12:34:56
```

## 🚀 배포 방법

### 로컬 테스트
```bash
python3 kiosk_server.py
```

### 외부 접속
서버가 `0.0.0.0:6027`로 실행되므로 같은 네트워크의 다른 기기에서도 접속 가능:
- **로컬**: http://localhost:6027
- **외부**: http://[서버IP]:6027

### 프로덕션 배포
- Gunicorn + Nginx 사용
- 도메인 연결
- SSL 인증서 적용