# c/ — 순수 C(C99) 구현

파이썬으로 검증한 판별 파이프라인을 외부 라이브러리 없이 순수 C로 구현한 것.
Arduino(.ino, C++) 버전과 별개로, ESP-IDF(공식 C 프레임워크)나 다른 MCU로 갈 때
그대로 쓸 수 있고, PC에서도 컴파일·검증된다.

| 파일 | 내용 |
|---|---|
| `edgealimi.h` | 공개 API + 파라미터 (파이썬 `ACTIVE_FEATURES`·`BaselineDetector` 기본값과 동일) |
| `edgealimi.c` | FFT(radix-2 DIT 직접 구현) + 특징 추출 + 3σ 판별기 + 연속성 필터 + 드리프트 경고 |
| `validate_main.c` | PC용 대조 검증 하니스 — 파이썬 정답값(`test_vectors.h`)과 비교 |

## .ino(C++) 버전과의 관계

- 로직·파라미터는 동일 (단일 출처: `src/` 파이썬 코드)
- .ino는 arduinoFFT(C++ 템플릿) 사용, 이쪽은 FFT까지 직접 구현 → **의존성 0**
- 센서 드라이버(MPU-6050 I2C 등)는 플랫폼 종속이라 여기 없음 —
  ESP-IDF에서는 `driver/i2c.h`로 별도 작성 후 `ea_extract_features()`에 연결

## PC에서 검증 실행

```sh
# repo 루트에서 (gcc 또는 MinGW 필요)
python -m tools.export_test_vectors        # 정답 파일 갱신 (선택)
gcc -std=c99 -O2 -Wall c/validate_main.c c/edgealimi.c -o validate -lm
./validate                                 # 종료코드 0 = 전체 통과
```

검증 항목: ① 특징 추출(FFT+대역에너지+RMS+회전추정, 상대오차 2% 이내)
② baseline 평균/표준편차 ③ 3σ 판별 불리언(정확 일치)
④ 상태관리(연속성 필터 미충족 억제, 재보정 후 원본 baseline 보존)

## ESP-IDF 프로젝트에 넣기

```
my_project/
├── main/
│   ├── main.c            ← app_main(): 센서 읽기 → ea_* 호출
│   ├── edgealimi.c       ← 이 폴더에서 복사 (또는 컴포넌트로 등록)
│   ├── edgealimi.h
│   └── CMakeLists.txt    ← idf_component_register(SRCS "main.c" "edgealimi.c" ...)
```

주의: `ea_compute_fft_magnitudes`/`ea_extract_features`는 내부 정적 버퍼를 써서
**재진입 불가** — 여러 태스크에서 동시에 부르면 안 됨 (단일 감시 태스크에서만 호출).
baseline 영속화(NVS)는 플랫폼 종속이라 이 모듈에 없음 — ESP-IDF에서는 `nvs_flash`로
`ea_detector_t`의 `means/stds/original_means/has_original_baseline`을 저장·복원할 것
(원본 baseline이 재부팅에 날아가면 위험 정상화 방지가 리셋된다).
