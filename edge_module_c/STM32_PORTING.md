# STM32 이식 가이드 — ESP32와 구분해야 할 부분

대상: `edge_module_c/core/` (v2 알고리즘) 를 STM32 로 옮길 때
기준 커밋: `716b9f3` (melajin/main) — v2 비율 정규화 · log-space 3σ · 센서 대역 상한 반영본

---

## 0. v2 적용 상태 (이식 시작 전 확인 사항)

이식의 출발점이 되는 코드가 실제로 v2인지 먼저 확인했다. 검증 결과:

| 확인 항목 | 결과 |
|---|---|
| `core/em_features.c` 비율 정규화 | 적용됨 — `e_total` 로 나눔 (H1/H2/H3/HF 전부) |
| `core/em_detector.c` log-space 3σ | 적용됨 — `EM_LOG_EPS 1e-6f`, 학습·판정·스냅샷 전부 같은 log 공간 |
| `core/em_config.h` 센서 대역 상한 | 적용됨 — `sensor_bw_hz` 필드 신설, HF 상한 = `min(Nyquist, sensor_bw_hz - 5)` |
| `core/` ↔ `esp32/edge_alimi/` 사본 동기화 | 12개 파일 전부 동일 (drift 없음) |
| 거동 검증 (`pc_test/main.c`) | **ALL PASS** (실패 0건) |
| 이식 대조 검증 (`pc_test/validate_v2.c`) | **PASS** — 최대 상대오차 2.838e-06 (float32 한계 3.808e-06 이내) |
| 스냅샷 버전 | `EM_SNAPSHOT_VERSION = 2` — v1 baseline 로드를 명시적으로 거부 |

**따라서 STM32 이식은 v2를 정본으로 시작한다.** v1 (절대 에너지 · 선형 3σ) 로
되돌아간 코드가 섞이면 검증 수치(불평형 90.2% / 정렬불량 82.5%)의 근거가 무효가 된다.

---

## 1. 이식 경계선 — 무엇이 갈리는가

```
┌─────────────────────────────────────────────┐
│  core/  (12파일)  ── 그대로 간다             │  ← 표준 C99 + <math.h>/<string.h>/<stdio.h> 뿐
│  em_fft / em_features / em_detector /       │     외부 라이브러리 의존 0
│  em_rotation / em_pipeline / em_config      │
└─────────────────────────────────────────────┘
                    ↕  이 경계가 전부다
┌─────────────────────────────────────────────┐
│  edge_alimi.ino (466줄)  ── 전면 재작성      │  ← Arduino/ESP-IDF API 에 밀착
│  I2C · 타이밍 · NVS · Wi-Fi · WebServer     │
└─────────────────────────────────────────────┘
```

`core/` 의 include 를 전수 확인한 결과 ESP32 전용 헤더는 하나도 없다. 즉 **알고리즘
재작성은 없고, 재작성 대상은 펌웨어 층(.ino) 하나다.** 아래는 그 층에서 ESP32와
STM32가 갈리는 지점을 항목별로 정리한 것이다.

---

## 2. 항목별 대비표

| # | 항목 | ESP32 (현재) | STM32 (이식 시) | 난이도 |
|---|---|---|---|---|
| 1 | 빌드 | Arduino IDE, `.ino` | STM32CubeIDE / CMake + arm-none-eabi-gcc, `main.c` | 중 |
| 2 | FPU | 240MHz Xtensa, 단정도 FPU 내장 | **MCU 선택에 종속** (§3) | **상** |
| 3 | RAM | 520KB — 무신경 가능 | 약 17.8KB 정적 필요 (§4) | 중 |
| 4 | `printf` 부동소수 | 기본 활성 | **newlib-nano 기본 비활성** (§5) | 함정 |
| 5 | 시각 | `millis()` / `micros()` | `HAL_GetTick()` / DWT CYCCNT 또는 TIM | 하 |
| 6 | I2C | `Wire` (Arduino) | `HAL_I2C_Mem_Read/Write` | 하 |
| 7 | 샘플 동기 | MPU DATA_RDY 폴링 | 폴링 유지 또는 EXTI+DMA (§6) | 중 |
| 8 | baseline 영속화 | NVS (`Preferences`) | 내장 Flash 직접 쓰기 / EEPROM 에뮬 (§7) | **상** |
| 9 | 통신 | Wi-Fi + WebServer + 콘솔 내장 서빙 | **무선 없음 → 구조 변경** (§8) | **상** |
| 10 | 명령 인터페이스 | HTTP POST + Serial 문자 | UART 문자 명령만 | 하 |
| 11 | 워치독 | 태스크 WDT 자동 | IWDG 명시 설정 + 리프레시 (§9) | 중 |
| 12 | 힙 | `ESP.getFreeHeap()` health 필드 | 힙 미사용(전부 정적) → 필드 제거/대체 | 하 |
| 13 | 경보 출력 | `digitalWrite(ALARM_PIN)` | `HAL_GPIO_WritePin` | 하 |
| 14 | 실행 문맥 | Arduino loop = FreeRTOS 태스크 (Wi-Fi 가 별 태스크) | 베어메탈 단일 루프 → **지터 감소(이점)** | — |

---

## 3. FPU — 이식 성공/실패를 가르는 첫 갈림길

`core/` 는 전부 `float` 로 작성되어 있고 `sqrtf/logf/cosf/sinf/fabsf` 를 12곳에서 쓴다.
이는 **Cortex-M4F/M7 의 단정도 FPU 와 정확히 맞는 설계**다. 하지만:

| MCU 계열 | FPU | 판정 |
|---|---|---|
| STM32F4 (F401/F411/F407), F7, H7, G4, L4 | 단정도 하드웨어 FPU | **적합** |
| STM32F0, F1(F103), G0, L0 | FPU 없음 → 소프트 float | **부적합** — 1024점 FFT 시간이 수십 배로 늘어남 |

빌드 플래그에 **반드시** 다음이 들어가야 한다. 빠뜨리면 FPU 가 있어도 소프트 float 로 컴파일된다:

```
-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
```

### double 승격 주의

- `em_fft.c` 의 `M_PI` 는 double 상수지만 `-2.0f * (float)M_PI / (float)len` 로 **이미 float 캐스트**되어 있어 안전하다.
- 반면 `em_detector_t` 의 Welford 누적기는 **의도적으로 `double`** 이다:
  ```c
  double w_mean[EM_NUM_FEATURES];
  double w_m2[EM_NUM_FEATURES];
  ```
  Cortex-M4F 는 배정도 FPU 가 없어 이 연산이 소프트웨어 에뮬레이션된다. 다만 이 코드는
  **학습 단계(`em_detector_fit_add`, 총 120윈도)에서만** 돌고 감시 루프에는 없다.
  윈도당 double 연산이 열 회 수준이라 실사용 영향은 무시할 만하다.
  **정밀도를 이유로 float 로 낮추지 말 것** — 분산 누적의 수치 안정성이 목적이다.
  (H7 등 배정도 FPU 보유 MCU면 그대로 이득)

---

## 4. RAM 예산

측정한 정적 소요량:

| 버퍼/구조체 | 크기 | 위치 |
|---|---|---|
| `s_re[1024]`, `s_im[1024]` (FFT 작업버퍼) | 8,192 B | `em_fft.c` static |
| `mag[512]` (스펙트럼) | 2,048 B | `em_pipeline.c` static |
| `g_sig[1024]` (샘플 윈도) | 4,096 B | 펌웨어 층 |
| `em_pipeline_t` (config+rotation+detector) | 2,712 B | 전역 |
| `em_output_t` + JSON 버퍼 | 약 712 B | 전역 |
| **합계** | **약 17.8 KB** | |

- **STM32F103 (20KB RAM) 은 불가** — 스택/HAL 여유가 안 나온다.
- 최소 권장: **64KB 이상** (F401RE 64KB, F411 128KB, F407 192KB).
- F4 의 CCMRAM(64KB, DMA 접근 불가)에 `s_re`/`s_im`/`mag` 를 배치하면 일반 SRAM 이
  넉넉해진다. 단 **`g_sig` 는 CCM 에 두지 말 것** — DMA 로 센서를 읽는 구성(§6)이면
  DMA 가 접근할 수 없다.
- 힙은 전혀 쓰지 않으므로 `malloc` 없는 링커 설정이 가능하다.

---

## 5. `printf` 부동소수 — 조용히 깨지는 함정

`em_output_to_json()` 은 `snprintf` 로 `%.3f`, `%.6g`, `%.1f` 를 쓴다.
STM32 툴체인 기본값인 **newlib-nano 는 부동소수 포맷이 링크에서 빠져 있어**,
그대로 두면 컴파일·링크는 성공하지만 런타임에 실수 필드가 정상 출력되지 않는다.
판정 결과 JSON 이 통째로 무의미해지는데 **에러가 안 난다**.

**해결**: 링커 플래그에 추가

```
-u _printf_float
```

(STM32CubeIDE: Project Properties → C/C++ Build → Settings → MCU Settings →
 "Use float with printf from newlib-nano" 체크)

코드 크기가 늘어난다. 이를 피하려고 `em_output_to_json` 을 정수 기반 고정소수점
포맷터로 교체하는 방법도 있으나, **그러면 파이썬/ESP32 와 출력 문자열이 달라져
대조 검증 경로가 끊긴다.** 플래그를 켜는 쪽을 권장한다.

---

## 6. 샘플링 동기 — v2 설계 의도를 지킬 것

현재 `acquire_window()` 는 **`micros()` 페이싱이 아니라 MPU 의 DATA_RDY 상태 폴링**으로
샘플을 읽는다. 이건 v2에서 의도적으로 바꾼 부분이다 — MCU 클럭과 센서 클럭의
비트(beat) 현상으로 생기던 중복/누락 샘플과 지터를 없애기 위함이다.

**STM32 이식 시 이 원칙을 깨지 말 것.** 두 가지 선택지:

- **(A) 폴링 그대로** — `HAL_I2C_Mem_Read` 로 `REG_INT_STATUS` 를 읽어 bit0 확인.
  가장 단순하고 ESP32 동작과 1:1 대응되어 대조 검증이 쉽다. 첫 이식은 이쪽 권장.
- **(B) EXTI + DMA** — MPU INT 핀을 GPIO EXTI 에 물리고 인터럽트에서 I2C DMA 읽기.
  CPU 점유가 낮아지지만 `acquire_window()` 의 블로킹 구조가 사라지므로
  더블버퍼 + "윈도 채워짐" 플래그로 파이프라인 호출 시점을 재설계해야 한다.
  ESP32 는 INT 핀 배선이 불필요했지만 (B) 는 **배선이 추가된다** — 하드웨어 차이.

어느 쪽이든 `acquire_window` 가 반환하는 **실측 fs 를 `cfg.sample_rate_hz` 에 반영하는
로직은 반드시 옮겨야 한다.** bin 해상도가 여기서 나오고, v2의 고조파 대역 반폭
계산이 전부 이 값에 걸려 있다.

또한 수집 중에는 통신 처리를 하지 않는다는 규칙(ESP32에서 HTTP 를 수집 중 처리하지
않는 이유)도 그대로 유효하다. STM32 에서는 UART 송신을 수집 구간 밖으로 빼면 된다.

---

## 7. baseline 영속화 — 가장 성격이 다른 부분

ESP32 는 NVS(`Preferences`)가 키-값 저장 + 마모 평준화를 다 해준다. STM32 에는 이에
대응하는 것이 **없다.** 저장 대상 자체는 작다:

- `em_snapshot_t` = **76바이트** (magic / version / n_features / mean[5] / stdv[5] /
  original_means[5] / 플래그 / checksum)
- 별도로 `rated_rpm` (float 4B) — 저장 당시 정격과 현재 정격이 다르면 baseline 을
  거부하는 로직이 붙어 있다. **이 규약을 같이 옮기지 않으면 다른 설비의 baseline 을
  그대로 물고 감시를 시작한다.**

구현 선택지:

| 방식 | 장점 | 주의 |
|---|---|---|
| 내장 Flash 마지막 섹터 직접 쓰기 (`HAL_FLASH_*`) | 부품 추가 없음 | **F4 섹터는 16~128KB** — 76B 쓰자고 섹터 전체를 소거. 마모 관리 필요 |
| X-CUBE-EEPROM (Flash EEPROM 에뮬레이션) | 마모 평준화 제공 | 미들웨어 추가, 페이지 2개 점유 |
| 외부 I2C EEPROM (24LC 계열) | 바이트 단위 쓰기, 마모 무관 | 부품·배선 추가 (단가 제약 검토 필요) |

**쓰기 빈도가 낮다는 점이 이 문제를 완화한다** — baseline 저장은 학습 완료 시 1회,
정격 변경 시 1회뿐이며 매 윈도 쓰기가 아니다. 따라서 내장 Flash 직접 쓰기로도
실사용 수명은 충분하다. 다만 **쓰기 도중 정전 시 baseline 이 깨지는 경로**는 ESP32
NVS 보다 노출이 크다. `em_snapshot_t` 에 checksum 이 이미 있어 손상 감지는 되지만
(손상 시 로드 거부 → 재학습 요구), 무손실을 원하면 A/B 두 슬롯 교대 쓰기를 권한다.

### 스냅샷 바이너리 호환성

ESP32 에서 학습한 baseline 을 STM32 로 그대로 옮길 수 있는지 확인했다:

- 멤버가 전부 고정폭 타입(`uint32_t`/`uint16_t`/`uint8_t`/`float`)이고 포인터·`size_t`·
  `long` 이 없다. `_pad` 도 명시되어 있다 → **양쪽에서 76바이트로 동일**
- 엔디안: Xtensa LE / ARM Cortex-M LE → 동일
- 부동소수: 양쪽 IEEE-754 binary32 → 동일

**결론: 이식 가능하다.** 단 `EM_SNAPSHOT_VERSION = 2` 이므로 v1 시절 저장분은
자동 거부된다(의도된 동작). 실무적으로는 센서 개체차·장착 방향이 달라지면 baseline
자체가 의미를 잃으므로, **MCU 를 바꿨으면 재학습하는 것이 원칙**이고 위 호환성은
디버깅·대조용으로만 쓰는 게 맞다.

---

## 8. Wi-Fi 부재 — 구조적으로 가장 큰 차이

ESP32 펌웨어는 Wi-Fi STA + `WebServer(80)` + `dashboard_html.h` 내장 서빙까지
전부 안고 있다. 대시보드 HTML 을 펌웨어에 임베딩하는 도구(`tools/embed_dashboard.py`)와
상태변경 API 보호용 `API_KEY` 도 여기 딸린 구조다. **STM32 에는 무선이 없으므로 이
블록 전체가 성립하지 않는다.**

선택지:

- **(A) UART / USB CDC 로 JSON 라인만 송출, 콘솔은 Pi 가 서빙** — **권장**
  - 3계층 아키텍처상 ESP32의 역할은 원래 **"계산기"** 이고 콘솔 서빙은 **"도서관"(Pi)**
    의 몫이다. ESP32 펌웨어가 콘솔까지 서빙한 건 Pi 없이도 돌게 하려던 편의였다.
  - 따라서 STM32 이식은 오히려 **원안 아키텍처에 더 가까워진다** — 후퇴가 아니다.
  - `WebServer` · `dashboard_html.h` · `embed_dashboard.py` · `API_KEY` 를 전부 드롭.
    HTTP 미노출이라 `API_KEY` 인증 요구도 함께 사라지고, 대신 **UART 명령 파서가
    유일한 상태변경 경로 = 보안 경계**가 된다.
  - `em_output_to_json()` 은 그대로 재사용 — 출력이 시리얼이든 HTTP 든 같은 한 줄이다.
- (B) ESP-01 등을 AT 모뎀으로 부착 — Wi-Fi 는 살지만 ESP32 를 결국 하나 더 쓰는 셈이라 부품 논리가 어긋남
- (C) W5500 이더넷 + LwIP — 유선 가능한 현장에서만. 망분리 포지셔닝과는 오히려 잘 맞음

### 드롭되는 것 / 유지되는 것

| ESP32 기능 | STM32 (A안) |
|---|---|
| `WebServer` + `/data` `/health` 엔드포인트 | 드롭 → UART JSON 라인 |
| 콘솔 HTML 펌웨어 내장 서빙 | 드롭 → Pi 가 서빙 |
| `POST /learn` `/config` `/baseline/clear` | UART 명령 `l` / `r<rpm>` / `c` 로 대응 (이미 존재) |
| `API_KEY` 쿼리 인증 | 불필요 (HTTP 미노출) |
| `WiFi.RSSI()`, `ESP.getFreeHeap()` health 필드 | 제거 또는 스택 하이워터마크로 대체 |
| 센서 장애 격리 · 클리핑 감지 (`sfault`/`clip`) | **유지** — 하드웨어 무관 로직 |

---

## 9. 워치독 — 블로킹 구간과의 충돌

ESP32 는 Arduino loop 이 FreeRTOS 태스크라 태스크 WDT 가 자동으로 붙는다.
STM32 베어메탈에서는 **IWDG 를 직접 설정하고 직접 리프레시**해야 한다.

주의점: `acquire_window()` 는 1024 샘플을 다 채울 때까지 블로킹한다. 1kHz 목표면
**약 1.02초**, I2C 실패로 3ms 타임아웃이 반복되면 그보다 길어질 수 있다.
**IWDG 리로드 주기를 이 최악값보다 길게 잡거나, 수집 루프 안에서 리프레시**해야
정상 동작 중에 리셋이 걸리지 않는다. 수집 루프 안 리프레시가 더 안전하지만,
I2C 가 완전히 죽었을 때 워치독 리셋과 센서 장애 격리 중 **어느 쪽을 우선할지**
결정하고 문서화할 것.

---

## 10. v2 고유 주의점 (알고리즘 쪽)

이식하면서 **v2의 세 가지 수정이 무효화되지 않도록** 특히 확인할 것.

### 10-1. 비율 정규화 — 센서 스케일에 관대해졌다 (단, RMS 제외)

v2에서 대역 에너지를 전체 에너지 대비 비율로 바꾼 덕분에, `harmonic1~3_ratio` 와
`high_freq_ratio` 는 **분자·분모에서 스케일이 상쇄되어 센서 감도·ADC 스케일이 달라도
그대로 유효**하다. 이식 관점에서 큰 이점이다.

**예외는 `EM_F_RMS` 하나** — 유일한 절대량 특징이다. ESP32 구현은 MPU-6050 ±4g
풀스케일 기준으로 g 단위 환산한다. STM32 에서 **다른 센서나 다른 풀스케일을 쓰면
RMS 값의 물리 단위가 달라지고**, 가동/정지 판별 SNR 임계값과 절대 RMS 임계값을
전부 재설정해야 한다. (이 두 파라미터는 애초에 실데이터 대기 중인 유보 항목이므로,
센서를 바꾸면 그 재측정을 STM32 기준으로 하면 된다.)

### 10-2. log-space 3σ — 연산 부담과 스냅샷 공간

- 판정/학습이 `logf(x + 1e-6f)` 공간에서 이뤄진다. 윈도당 `logf` 호출은 5회뿐이라
  M4F 에서 무시할 수준이다. **FPU 없는 MCU 라면 여기도 비용이 붙는다** (§3에서
  이미 배제한 이유가 하나 더 늘어나는 셈).
- **스냅샷의 mean/stdv 는 log 공간 값**이다. 저장·복원 코드를 새로 쓰면서 선형
  공간으로 되돌리는 변환을 넣지 말 것 — 넣으면 조용히 v1 거동으로 회귀한다.

### 10-3. `sensor_bw_hz` — 센서를 바꾸면 반드시 갱신

`sensor_bw_hz = 260.0f` 는 **MPU-6050 DLPF=0 설정의 유효 대역**이라는 전제에서 나온
값이다. HF 특징의 상한이 `min(Nyquist, sensor_bw_hz - 5)` 로 잡히고, 이게 v2에서
"DLPF 감쇠 구간의 센서 노이즈를 특징으로 재던" 문제를 고친 부분이다.

STM32 에서 **다른 가속도계(ADXL345, LIS3DH, IIS3DWB 등)를 쓰면 이 값을 그 센서의
유효 대역으로 반드시 바꿔야 한다.** 안 바꾸면:

- 실제 대역이 260Hz보다 좁은 센서 → 여전히 노이즈를 특징으로 잼 (v2 수정 무효화)
- 실제 대역이 더 넓은 센서 → 쓸 수 있는 정보를 버림

같은 MPU-6050 을 STM32 에 그대로 물린다면 260 유지가 맞다. **단 DLPF 설정값을
ESP32 초기화 코드와 동일하게 맞췄는지 확인**해야 이 전제가 성립한다.

---

## 11. 검증 절차 — 하드웨어 없이 먼저 증명한다

이 프로젝트의 검증 축은 두 개이고 서로 대체 불가다. 이식 후에도 둘 다 돌려야 한다.

1. **수치 정합성 (`pc_test/validate_v2.c`)** — 이식 정합성의 단일 증거.
   `v2_vectors.h` 에 파이썬 float64 정답값이 내장되어 있어 **외부 입력 없이 보드
   단독 실행이 가능**하다. STM32 로 크로스 컴파일해 보드에서 돌리고 결과를 UART 로
   받는 것이 이식 검증의 첫 단계여야 한다.
   - 기준: PC 에서 최대 상대오차 **2.838e-06** (허용치 1.0e-03)
   - STM32 에서도 같은 자릿수가 나와야 한다. 크게 벗어나면 `-mfloat-abi` 설정이나
     컴파일러 최적화를 의심할 것.
2. **거동 검증 (`pc_test/main.c`, 28항목)** — 오탐 / 감지지연 / 드리프트 / 정지 오경보
   차단 / 스냅샷 복원까지. RAM 이 허용하면 보드에서, 아니면 최소한 같은 컴파일러
   설정으로 호스트에서 재실행.
3. **실측 fs 확인** — `acquire_window()` 반환값이 목표 1000Hz 대비 얼마나 나오는지.
   여기가 어긋나면 위 두 검증이 다 통과해도 현장에서 스펙트럼이 틀어진다.

> `-ffast-math` 는 켜지 말 것. `logf`/`sqrtf` 의 결과가 달라져 파이썬 대조 검증이
> 깨지고, 이 프로젝트에서 이식 정합성을 주장할 근거가 사라진다.

---

## 12. 이식 체크리스트

**툴체인 / 빌드**

- [ ] MCU 가 단정도 FPU 보유 (F4/F7/H7/G4/L4) — F1/F0/G0/L0 배제
- [ ] RAM 64KB 이상
- [ ] `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` 설정
- [ ] `-u _printf_float` 링커 플래그 (또는 CubeIDE 체크박스)
- [ ] `-ffast-math` **비활성** 확인
- [ ] `core/` 12파일을 수정 없이 그대로 프로젝트에 추가 (헤더에 `extern "C"` 이미 있음)

**하드웨어 층 재작성**

- [ ] I2C: `Wire` → `HAL_I2C_Mem_Read/Write`, 400kHz timing 계산, 타임아웃 5ms 대응
- [ ] MPU 초기화 레지스터 값을 ESP32 코드와 **동일하게** (특히 DLPF, `SMPLRT_DIV`, ±4g)
- [ ] DATA_RDY 동기 샘플링 유지 (폴링 A안 권장)
- [ ] 실측 fs 를 `cfg.sample_rate_hz` 에 반영하는 로직 이식
- [ ] `micros()` → DWT CYCCNT 또는 프리러닝 TIM (32비트 랩어라운드 뺄셈은 그대로 유효)
- [ ] `millis()` → `HAL_GetTick()`
- [ ] 경보 GPIO 출력
- [ ] 클리핑 감지(`ACCEL_CLIP_RAW`) · I2C 실패 카운트 · 센서 장애 격리 이식

**영속화**

- [ ] Flash 쓰기 방식 결정 (직접 / EEPROM 에뮬 / 외부 EEPROM)
- [ ] `rated_rpm` 불일치 시 baseline 거부 규약 이식
- [ ] checksum 검증 · 손상 시 로드 거부 동작 확인
- [ ] (선택) A/B 슬롯 교대 쓰기로 정전 내성 확보

**통신 / 구조**

- [ ] Wi-Fi · WebServer · 대시보드 임베딩 블록 전체 드롭
- [ ] UART JSON 라인 송출 (`em_output_to_json` 재사용)
- [ ] UART 명령 파서 (`l` 학습 / `r<rpm>` 정격 / `c` baseline 삭제)
- [ ] health 필드에서 `getFreeHeap` / `RSSI` 제거 또는 대체
- [ ] IWDG 설정 + 수집 루프 내 리프레시 정책 결정

**v2 무결성**

- [ ] `sensor_bw_hz` 가 실제 사용 센서의 유효 대역과 일치
- [ ] 스냅샷을 log 공간 값 그대로 저장 (선형 역변환 삽입 금지)
- [ ] 센서 / 풀스케일 변경 시 RMS 관련 임계값 재설정 계획
- [ ] `validate_v2` 보드 실행 → 상대오차 1e-03 이내 PASS
- [ ] 거동 검증 28항목 PASS
