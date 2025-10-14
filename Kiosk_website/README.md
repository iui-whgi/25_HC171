# Kiosk Firebase Hosting 배포 가이드 (Windows PowerShell)

이 문서는 현재 디렉토리(`kiosk`) 기준으로 Windows PowerShell에서 Firebase Hosting에 배포하는 방법을 정리합니다.

## 사전 준비
- Node.js 및 npm 설치
- Firebase CLI 설치: `npm i -g firebase-tools`
- Firebase 로그인(최초 1회): `firebase login`
- 프로젝트에 `firebase.json` 존재 확인

## 배포
PowerShell에서 이 폴더로 이동한 뒤 아래 명령을 실행합니다.

```powershell
# 폴더 이동 (예시)
cd "C:\Users\stomm\Desktop\2025 Summer Vacation\kiosk"

# 비대화식 최신 CLI로 Hosting만 배포
npx --yes firebase-tools@latest deploy --only hosting --non-interactive
```

배포가 성공하면 출력 하단에 Hosting URL이 표시됩니다. 예: `https://subwaykiosk-6b13b.web.app`

## 자주 쓰는 명령
```powershell
# 로그인 (최초 1회 또는 계정 변경 시)
firebase login

# 프로젝트 선택/초기화 (이미 설정되어 있으면 생략 가능)
firebase init hosting

# 로컬에서 미리보기 (Emulator)
firebase emulators:start --only hosting

# 최신 CLI로 배포 (권장)
npx --yes firebase-tools@latest deploy --only hosting --non-interactive
```

## 참고
- 이 프로젝트는 `firebase.json`에 정의된 설정을 사용합니다.
- `public` 디렉토리를 별도로 사용하지 않고, 현재 디렉토리의 정적 파일을 배포하도록 설정되어 있을 수 있습니다. 필요 시 `firebase.json`의 `hosting.public` 값을 변경하세요.
- PowerShell에서는 명령 연결 시 `;`를 사용합니다. (`cmd`의 `&&`와 다릅니다.)
