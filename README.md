# 엣지알리미 — 소프트웨어 파이프라인 초기 설계도

저비용 엣지 AI 예지보전 모듈 "엣지알리미"의 소프트웨어(신호처리·이상탐지) 부분
초기 설계 코드입니다. 실제 ESP32 하드웨어와 센서 데이터가 준비되기 전, 로직 자체를
먼저 설계하고 검증하기 위한 목적으로 작성했습니다.

## 지금 상태 (정직하게 명시)

- 실제 하드웨어(ESP32, MPU-6050) 배선/연결: **아직 안 됨**
- 실제 정상/고장 진동 데이터: **없음**
- 확정된 것: 파이프라인 로직 설계 (FFT → 특징추출 → 통계 baseline 판별)

이 저장소는 데이터 없이도 로직을 먼저 짜고 시뮬레이션 신호로 검증하기 위한
**임시 구조**입니다. 실제 센서 데이터가 확보되면 `src/signal_generator.py`만
실제 데이터 리더로 교체하면 되고, 나머지 모듈은 그대로 재사용하도록 설계했습니다.

## 설치

```bash
git clone <this-repo-url>
cd edgealimi_predictive_maintenance
pip install -r requirements.txt
```

## 실행

각 모듈은 독립적으로 실행 가능합니다 (`python -m` 방식 사용, 프로젝트 루트에서 실행):

```bash
python -m src.signal_generator      # 1. 가짜 진동 신호 생성 확인
python -m src.feature_extraction    # 2. FFT + 특징추출 (정상 vs 고장 비교)
python -m src.baseline_detector     # 3. 통계 baseline 판별 로직 (핵심)
python -m src.reference_comparison  # 4. scikit-learn과 비교 (참고용)
python -m src.visualize             # 5. 그래프 4장 생성 (outputs/ 폴더에 저장)
```

## 파이프라인 구조

```
signal_generator.py      정상/고장 진동 신호 생성 (시뮬레이션)
        ↓
feature_extraction.py    FFT + 특징추출 (RMS, 첨도, 배음 에너지, 고주파 에너지)
        ↓
baseline_detector.py     통계 baseline 판별 + 연속성 필터 + 위험정상화 방지  ★핵심
        ↓
reference_comparison.py  scikit-learn Isolation Forest와 비교 (참고용, ESP32 미이식)
        ↓
visualize.py              위 결과를 그래프로 시각화
```

### `src/signal_generator.py`
실제 센서가 없는 상태에서, 사인파(회전 주파수 + 지터) + 배음 + 노이즈로 정상
진동을 흉내내고, 배음 진폭 증가·고주파 성분 추가로 고장을 흉내냅니다.
`generate_progressive_wear_sequence()`는 서서히 악화되는 마모 시퀀스를 생성해
위험 정상화(Normalization of Deviance) 방지 로직 검증에 씁니다.

**한계**: 수학적으로 흉내낸 신호이며 실제 물리적 고장이 아닙니다. 실제 데이터로
반드시 재검증이 필요합니다.

### `src/feature_extraction.py`
RMS, 첨도, 2차/3차 배음 대역 에너지, 고주파(250~400Hz) 대역 에너지를 계산합니다.
`peak_freq`/`peak_magnitude`는 FFT 해상도 한계로 변별력이 낮아 참고용으로만
남겨뒀습니다.

**주의**: ESP32에는 scipy가 없으므로 이 계산식들은 C로 재구현해야 합니다.

### `src/baseline_detector.py` (핵심 로직)
정상 데이터의 평균/표준편차로 baseline을 잡고 3-sigma 이탈 시 1차 의심 판정을
내립니다. 다음 두 가지 안전장치가 추가되어 있습니다.

- **연속성 필터링**: 연속 5개 윈도우 중 60% 이상이 의심 판정이어야 "확정 이상"으로
  격상합니다 (단발 노이즈로 인한 오탐 방지).
- **위험 정상화 방지**: 원본 baseline을 별도로 보관해, 현재 관측값이 원본 대비
  1.5배 이상 벌어지면 별도 경고를 냅니다 (서서히 나빠지는 걸 "새로운 정상"으로
  착각하는 것을 방지).

`ABSOLUTE_RMS_LIMIT = None`으로 절대 임계값 자리는 마련해뒀으나, 실제 데이터시트
확보 전까지는 비활성 상태입니다.

시뮬레이션 신호 기준 측정 결과: 오탐율 6%, 미탐율 0% (`fault_strength=0.5` 기준).
이 로직이 최종적으로 ESP32 C/C++ 코드로 이식될 대상입니다.

### `src/reference_comparison.py` (참고용, ESP32 미이식)
같은 데이터로 scikit-learn의 Isolation Forest 결과와 비교합니다. 배음 에너지
특징을 포함한 v2 특징 세트에서는 스케일링 여부와 무관하게 Isolation Forest도
잘 작동함을 확인했습니다. 그럼에도 `baseline_detector.py` 방식을 최종 채택한
이유는 성능이 아니라 **이식 용이성**입니다 — scikit-learn은 ESP32에 존재하지
않고, 통계 baseline 방식은 평균/표준편차 몇 개 숫자만 저장하면 되므로 이식
난이도가 훨씬 낮습니다.

### `src/visualize.py`
- `fig1_fft_spectrum.png`: 정상 vs 고장 주파수 스펙트럼
- `fig2_feature_comparison.png`: 정상/경미/심각 고장의 특징값 비교
- `fig3_progressive_wear.png`: 서서히 나빠지는 마모 과정의 특징값 추이
- `fig4_detection_confusion.png`: 고장 강도별 탐지 성공률 —
  시뮬레이션 기준 `fault_strength` 0.01~0.02 사이가 이 로직의 민감도 경계선입니다
  (0.05 이상은 100% 탐지, 0.01 이하는 거의 탐지하지 못함).

그림은 `outputs/` 폴더에 저장되며, 이 폴더는 재실행으로 다시 생성 가능하므로
버전관리 대상에서 제외했습니다 (`.gitignore` 참고).

## 다음 단계 (실제 데이터 확보 후)

1. ESP32 + MPU-6050으로 정상 상태 진동 raw 데이터를 시리얼로 수집
2. `src/signal_generator.py`를 실제 CSV/시리얼 읽기 함수로 교체
   (반환 형식 `(t, signal)` 튜플만 유지하면 다른 모듈 수정 불필요)
3. 인위적 고장(불균형 무게, 헐거운 베어링) 상태 데이터도 동일하게 수집
4. `baseline_detector.py`의 `ABSOLUTE_RMS_LIMIT`을 실제 데이터시트 기준값으로 채움
5. 실제 데이터로 `visualize.py` 그래프들을 재생성해 시뮬레이션 결과와 비교
6. 검증 끝난 로직을 의사코드로 정리해 C/C++로 이식
7. 이식된 ESP32 코드와 파이썬 코드가 같은 입력에 같은 출력을 내는지 대조 검증

## 알려진 한계

- 실제 이상탐지 성능은 검증되지 않았습니다 (전부 시뮬레이션 신호 기준입니다).
- 절대 임계값 로직은 자리만 있고 실제 값은 채워지지 않았습니다.
- 센서 부착 노이즈, 실측 환경 변수(온도 드리프트, 전원 노이즈 등)는 반영되지
  않았습니다.
- `fig4`에서 확인한 "탐지 가능한 최소 고장 강도"는 시뮬레이션 신호의 수학적
  설계에 따른 결과이며, 실제 베어링 마모·팬 불균형이 이 강도에 대응하는지는
  알 수 없습니다.

이 저장소는 "이미 작동하는 시스템"이 아니라 **"실제 데이터가 들어올 자리를
미리 만들어 둔 설계도"**입니다.
