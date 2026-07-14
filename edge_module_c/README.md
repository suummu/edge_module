# 엣지알리미 C/C++ 시스템 (edge_module_c)

파이썬 설계도(`suummu/edge_module`)를 ESP32 실기기용 C 로 정밀 포팅한 것.
**테스트 데이터 없이도 범용 배포 가능** — 설비 명판 정격 RPM 하나만 입력하면,
나머지(회전수 탐색범위·고조파 대역·고주파 경계)는 전부 상대값으로 자동 파생되고,
정상 기준선은 현장에서 자가 학습한다.

## 구조 (3계층: 계산기 / 도서관 / 디스플레이)

```
core/        순수 C 코어 — ESP32와 PC에서 동일 컴파일 (단일 진실)
  em_config.[ch]     전 설정 집중. rated_rpm 이 유일한 필수 입력
  em_fft.[ch]        radix-2 FFT + Hann, 라이브러리 무의존
  em_rotation.[ch]   gap ① rotation_hz_estimate (탐색범위 제한 + 점프 거부)
  em_features.[ch]   ACTIVE_FEATURES 5종, 고조파 대역은 회전수 상대 배치
  em_detector.[ch]   3σ baseline + confirm 원형버퍼 + 드리프트 감시
  em_pipeline.[ch]   윈도 입력 → 판정 출력 + JSON 직렬화
esp32/edge_alimi/    Arduino 스케치 (계산기 계층) + core 사본
pc_test/             하드웨어 없는 검증 하니스 (시나리오 A/B 자동화)
pi/pi_server.py      라즈베리파이 도서관 계층 완성판 (수집+서빙+이력 API)
pi/pi_logger.py      수집만 필요할 때의 경량 대안
viz/dashboard.html   감시 콘솔 원본 (디스플레이 계층, 단일 파일)
tools/embed_dashboard.py  대시보드 → 펌웨어 헤더(gzip) 변환기
```

## 브라우저 접속 경로 — 파일 배포 없이 주소만 치면 콘솔이 뜬다

| 경로 | 주소 | 용도 |
|---|---|---|
| ESP32 단독 | `http://<esp32-ip>/` | 대시보드가 펌웨어에 gzip 내장(8.2KB). Pi 없이도 브라우저 접속만으로 라이브 감시. same-origin 이라 CORS/mixed-content 없음 |
| Pi 경유 (권장) | `http://<pi-ip>:8080/` | 라이브 + **저장 이력 복원**([저장 이력 불러오기] 버튼 → /history). 3계층 설계의 완성형 |
| **인터넷 공개 데모** | https://suummu.github.io/edge_module/ | 장비·설치 없이 아무 브라우저/폰에서 접속 — 데모 시뮬레이터 모드로 학습→감시→결함 주입→정지→드리프트 전체 흐름 시연 (HTTPS 라 PWA 오프라인 캐시까지 동작) |
| 파일 직접 | dashboard.html 열기 | 오프라인 데모/발표 백업 (데모 시뮬레이터 모드) |

콘솔은 로드 시 자기 출처의 /data 를 확인해 **장치 서빙이면 실장비 모드로 자동 연결**된다.
viz/dashboard.html 수정 후에는 `python3 tools/embed_dashboard.py` 로 펌웨어 헤더를 재생성하고,
공개 데모용 사본(`docs/index.html` + PWA 자산)도 viz/ 에서 다시 복사할 것 (viz/ 가 단일 원본).

### 모바일 앱 (PWA — 홈 화면 설치)

콘솔은 PWA 로 구성되어 폰에서 **앱처럼 설치·실행**된다 (스토어/APK 불필요):
- Android Chrome: `http://<pi-ip>:8080` 접속 → 메뉴 → "홈 화면에 추가"
- iOS Safari: 공유 → "홈 화면에 추가"
설치 후 전용 아이콘 + 전체화면(주소창 없음)으로 실행되며 모바일 레이아웃이 적용된다.
ESP32 단독 접속에도 manifest/아이콘이 내장되어 동일하게 설치 가능 (내장 총 12.2KB).
오프라인 앱 셸 캐시(sw.js)는 브라우저 정책상 보안 컨텍스트(HTTPS/localhost)에서만
동작한다 — LAN http 에서는 설치·전체화면은 되고 오프라인 캐시만 생략됨.
판정 데이터는 의도적으로 캐시하지 않는다: 감시 화면에 낡은 판정을 보여주는 것이
최악이므로, 통신 실패는 숨기지 않고 드러낸다.

## 빠른 시작

### 1) PC 검증 (하드웨어 불필요)
```bash
cd pc_test
gcc -O2 -Wall -Wextra -o em_test main.c ../core/em_*.c -lm
./em_test
```
검증 항목(24건 자동): 시나리오 A(정상 500윈도 확정 오탐 0건), 시나리오 B(4개
결함 유형 감지 지연 — 미탐 시 정직 기록), 드리프트 발화, **재-fit 후
original_means 보존**, 타 RPM(1800) + 회전수 ±8% 드리프트 강건성, JSON 직렬화,
**설비 정지/재가동 오경보 차단**, **baseline 스냅샷 저장/복원/손상 거부**,
**무분산 특징 z 폭발 억제**.

### 2) ESP32 배포
1. Arduino IDE 에서 `esp32/edge_alimi/edge_alimi.ino` 열기 (core 사본 포함됨)
2. `WIFI_SSID / WIFI_PASS / RATED_RPM_DEFAULT` 수정
3. 업로드 → 시리얼에서 IP 확인
4. `POST http://<ip>/learn?n=120` 으로 정상 기준선 학습 (~2분)
5. 다른 설비로 이식: `POST /config?rpm=<명판RPM>` 후 재학습 — 코드 수정 불필요

상태변경 API(/learn /config /baseline/clear)는 **POST 전용**이며, 펌웨어의
`API_KEY` 를 설정하면 `&key=<값>` 이 일치해야만 실행된다 (콘솔의 API 키 입력칸
사용). 미설정("") 시 검사 생략 — 데모 편의용이며 공유 LAN 투입 시 설정 권장.

core/ 를 수정했으면 `./sync_core.sh` 로 스케치 폴더 사본을 갱신할 것.
(사본이 어긋나면 파이썬에서 겪은 부분 전파 버그의 재판이 된다.)

### 3) 도서관 + 디스플레이
```bash
python3 pi/pi_server.py http://<esp32-ip> --port 8080 --outdir ./logs
# → 현장 누구든 브라우저에서 http://<pi-ip>:8080 접속 (설치·파일 배포 불필요)
```
pi_server 는 수집(JSONL 축적) + 콘솔 서빙 + ESP32 프록시 + /history 이력 API 를
한 프로세스로 수행한다. 장치가 죽으면 502 esp32_unreachable 로 정직하게 알린다.
대시보드는 인터넷/장비 없이도 "데모 시뮬레이터" 모드로 전체 흐름
(학습 → 감시 → 결함 주입 → 정지 → 확정 판정 → 드리프트) 시연 가능.

## 파이썬 대비 유지된 설계 결정

| 항목 | 내용 |
|---|---|
| ACTIVE_FEATURES | rms, harmonic1/2/3_energy, high_freq_energy — 순서·의미 동일. peak_freq/kurtosis 판별용 deprecated 유지 |
| 기준축 | 고조파 대역은 고정 Hz 가 아니라 rotation_hz_estimate 상대 배치 (기존 search_range_hz=(30,70) 하드코딩 문제 구조 해결) |
| 판정 | 3σ + anomaly_min_features + confirm_window(5) 중 4회 확정 |
| 드리프트 | 롤링 평균 vs original baseline. 구조 확정, drift_sigma/window 는 실기기 데이터 후 튜닝 예정 |
| 오탐/미탐 | 확정 판정의 최소 4윈도 지연은 오경보 억제를 위한 **설계된 트레이드오프** — 두 지표를 함께 보고 |

## 실무 투입 감사(audit)로 추가된 기능

| 결함 (심각도) | 수정 | 검증 |
|---|---|---|
| baseline 휘발 — 정전마다 2분 재학습 (치명) | em_snapshot_t 직렬화 + ESP32 NVS 자동 저장/부팅 복원. magic/version/checksum 으로 손상 로드 거부. 정격 RPM 변경 시 저장분 자동 무효화 | pc_test [8] |
| 설비 정지 = 오경보 (치명) | 회전 피크 SNR 가동/정지 판별 + 히스테리시스. 정지 중 판정·학습 중단, 재가동 시 판정 버퍼 리셋 | pc_test [7] |
| 센서 장애 침묵 (치명) | I2C 실패/타임아웃 감지, 윈도 2% 초과 실패 → SENSOR_FAULT 보고(판정 금지, 경보 접점 해제). ±4g 클리핑 1% 초과 → clip 플래그 | 펌웨어 |
| 정지/기동 중 학습 오염 (높음) | 학습은 가동 판별된 윈도만 수집, warmup_windows 만큼 기동 과도 폐기 | pc_test [7] |
| 절대 std 하한의 z 폭발 (높음) | min_rel_std (mean 의 2%) 상대 하한 | pc_test [9] |
| 샘플링 지터 / Wi-Fi 단절 / 경보 출력 부재 (중간) | 수집 루프에서 HTTP 제거(응답 최대 ~1초 지연은 의도된 트레이드오프), 10초 주기 자동 재연결, ALARM_PIN 무전압 접점(확정 이상에서만 구동), GET /health 현장 점검 API | 펌웨어 |

## gap ⑥ 체크리스트 반영 확인

- `deque(maxlen=N)` → 원형버퍼 + `flag_filled` 카운터. 가동 초기 미초기화
  슬롯을 읽지 않음 (`em_detector.c` 확정 판정부).
- `original_means` 는 `has_original_baseline` 플래그로 **최초 fit 1회만 기록**.
  pc_test [4] 가 재-fit 후 보존을 자동 검증 — 이게 깨지면 드리프트 감지가
  조용히 무력화되므로 코드 수정 시 반드시 테스트 재실행.

## 정직한 한계 (발표/문서에 그대로 쓸 것)

- MPU-6050 + I2C 실효 대역 ~500Hz → 타겟은 저주파 결함(불평형/정렬불량/이완).
  `high_freq_energy` 는 보조 지표이며 베어링 정밀 진단(1–20kHz 포락선 분석)이 아님.
- 회전수 추정은 A안(진동 1× 성분 근사): 명판 정격을 프리셋으로 쓰며
  PLC 연동이 없어 급격한 실시간 부하 변동 반영에는 한계.
- 현재 검증은 합성 신호 기반. 실기기 데이터 확보 후 드리프트 파라미터 튜닝과
  시나리오 A/B 재측정 필요.
- 정상-only 학습은 "무언가 이상하다"까지만 말한다 — 조기경보이지 진단이 아님.
  콘솔이 어떤 지표가 얼마나 이탈했는지 보여주는 것이 해석 공백의 완충이다.
- 가동/정지 판별은 회전 피크 SNR 기반: 배경 진동이 극심해 스펙트럼이 평탄한
  현장, 또는 인접 설비 진동이 넘어오는 현장에서는 run_snr_threshold 튜닝 필요.
- ESP32 실기기 컴파일/실측(fs 실효치, FFT 처리 시간, NVS 쓰기 수명)은 미검증.
  NVS 저장은 학습 완료 시 1회뿐이라 플래시 수명 문제는 없음.
