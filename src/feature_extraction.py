"""
feature_extraction.py

목적:
    signal_generator.py에서 만든 (또는 나중에 실제 센서에서 읽은) 시간 영역 신호를 받아서:
    1) FFT로 주파수 영역으로 변환
    2) 핵심 특징값(RMS, 첨도, 배음 에너지 등) 추출

    이 파일의 함수들은 입력이 "가짜 신호"든 "실제 센서 신호"든 상관없이 동작한다.
    즉 signal_generator.py만 실제 데이터 리더로 교체되면, 이 파일은 그대로 재사용 가능.

주의(실제 ESP32 이식 시 고려할 점, 지금은 파이썬으로만 검증):
    - np.fft, scipy.stats 같은 라이브러리는 C/ESP32에 없음
    - RMS, 첨도는 결국 for 루프 기반 직접 계산으로 재구현해야 함
    - 여기서는 "어떤 값이 왜 필요한지"를 파이썬으로 먼저 확정하는 것이 목적
"""

import numpy as np
from scipy.stats import kurtosis
from scipy.fft import fft, fftfreq

from . import signal_generator as gen

SAMPLE_RATE = gen.SAMPLE_RATE
WINDOW_SIZE = gen.WINDOW_SIZE


def compute_fft(signal, sample_rate=SAMPLE_RATE):
    """
    시간 영역 신호 -> 주파수 영역 변환.

    반환값:
        freqs: 각 주파수 성분의 실제 Hz 값 (양의 주파수만)
        magnitudes: 각 주파수 성분의 크기(진폭)
    """
    n = len(signal)
    fft_result = fft(signal)
    freqs = fftfreq(n, d=1 / sample_rate)

    # 실수 신호의 FFT는 대칭이므로 양의 주파수 절반만 사용
    half = n // 2
    freqs = freqs[:half]
    magnitudes = np.abs(fft_result[:half]) * 2 / n  # 정규화

    return freqs, magnitudes


def frequency_band_energy(freqs, magnitudes, center_hz, band_width_hz=5.0):
    """
    특정 주파수 대역(center_hz ± band_width_hz)의 에너지 합을 구한다.

    왜 필요한가:
        FFT 주파수 해상도(bin 간격)는 sample_rate / window_size로 정해진다.
        지금 설정(1000Hz / 256포인트)은 해상도가 약 3.9Hz라, "정확히 몇 Hz에서
        피크가 떴는가"라는 단일 값(peak_freq)은 회전속도가 조금만 흔들려도
        같은 bin에 계속 걸리거나 옆 bin으로 튀는 등 불안정할 수 있다.
        따라서 실제 판별에는 "이 대역 전체에 에너지가 얼마나 실렸는가"를
        같이 보는 것이 더 안정적이다. (엣지알리미 논리정리의
        "절대 위치가 아닌 상대적 위치/배수 판별" 원칙과 같은 맥락)
    """
    idx = (freqs >= center_hz - band_width_hz) & (freqs <= center_hz + band_width_hz)
    if not np.any(idx):
        return 0.0
    return float(np.sum(magnitudes[idx]))


def estimate_rotation_hz(freqs, magnitudes, search_range_hz=(30.0, 70.0)):
    """
    FFT 결과에서 이번 윈도우의 회전 주파수 "후보(candidate)"를 측정한다 -
    harmonic1/2/3_energy가 어디를 볼지 정하는 기준축(reference axis) 역할.

    peak_freq와 다른 점: peak_freq는 판별용 특징(feature)으로 썼다가 분산이
    거의 없어 폐기됐다(변별력 없음). 이 함수는 판별용이 아니라 "지금 회전수가
    몇 Hz인가"를 추정하는 것이 목적이라 역할이 다르므로 별개 함수로 둔다.

    search_range_hz: 정격 RPM(nameplate rated RPM) 기준 탐색 범위. FFT 전체에서
        최대 피크를 찾지 않고 이 범위 안에서만 찾아, 배음이나 노이즈 피크에
        낚이지 않게 한다.

    이 함수는 상태를 갖지 않는 순수 측정 단계이며, 노이즈인지 실제 변화인지의
    판단은 RotationTracker가 맡는다 (C의 em_rotation_measure / em_rotation_accept
    분리와 동일한 구조). 이전에는 한 함수가 측정과 점프 필터를 겸했는데,
    상태를 들 곳이 없어 한 번 크게 튄 뒤로는 이후 모든 후보가 계속 거부되어
    기준축이 옛 값에 영구 고정되는 결함이 있었다.

    한계(A-series): 정격 RPM을 preset 탐색범위로 쓰지만, PLC 연동이 없어
    실시간 부하 변동(운전점 자체의 이동)은 반영하지 못한다.
    """
    idx = (freqs >= search_range_hz[0]) & (freqs <= search_range_hz[1])
    if not np.any(idx):
        return None

    band_freqs = freqs[idx]
    band_mags = magnitudes[idx]
    return float(band_freqs[np.argmax(band_mags)])


class RotationTracker:
    """
    회전 주파수 기준축의 상태 추적기 (C의 em_rotation_t + em_rotation_accept).

    노이즈는 산발적이고 실제 속도 변화는 지속된다는 성질로 둘을 가른다:
      - max_jump_hz 이내 변화                        -> 즉시 추종
      - 초과하되 매번 다른 곳에 튐                    -> 거부, 이전 값 유지 (노이즈)
      - 초과하되 같은 자리에 relock_windows회 연속    -> 새 운전점으로 수용

    relock 단계가 없으면, 가변전압/VFD로 회전수를 한 번에 크게 바꿨을 때
    이후 모든 후보가 계속 거부되어 기준축이 옛 값에 영구 고정된다. 그러면
    하모닉 대역이 엉뚱한 중심에 놓여 harmonic1/2/3_energy가 전부 무의미해진다.

    rejected는 "지금 락이 걸려 있나"를 나타내는 상태값이다 - 정상 추종 시
    0으로 리셋되므로 단조 증가하는 누적 카운터가 아니다.
    """

    def __init__(self, max_jump_hz=5.0, relock_windows=5):
        self.hz = None            # 현재 확정 추정치
        self.pending_hz = None    # 거부 중인 후보 (연속성 확인용)
        self.rejected = 0         # 같은 자리에서 연속 거부된 횟수
        self.max_jump_hz = max_jump_hz
        self.relock_windows = relock_windows

    def accept(self, candidate_hz):
        """후보를 점프 필터에 통과시켜 기준축을 갱신하고 현재 추정치를 반환."""
        if candidate_hz is None:
            return self.hz

        if self.hz is None:                     # 최초 추정 - 무조건 채택
            self.hz = candidate_hz
            return self.hz

        if abs(candidate_hz - self.hz) <= self.max_jump_hz:
            self.hz = candidate_hz
            self.rejected = 0                   # 정상 추종 - 누적 거부 해제
            self.pending_hz = None
            return self.hz

        # 점프 - 같은 자리에 연속으로 나타나는 후보만 실제 변화로 인정
        if (self.pending_hz is not None
                and abs(candidate_hz - self.pending_hz) <= self.max_jump_hz):
            self.rejected += 1
        else:
            self.rejected = 1
        self.pending_hz = candidate_hz

        if self.rejected >= self.relock_windows:
            self.hz = candidate_hz              # 지속된 변화 - 새 운전점 수용
            self.rejected = 0
            self.pending_hz = None
        return self.hz


def extract_features(signal, sample_rate=SAMPLE_RATE, rotation=None):
    """
    하나의 신호 구간(윈도우)에서 핵심 특징값을 뽑아낸다.

    v2 수정 사항 (첫 버전의 문제점 보완):
      - 첫 버전은 peak_freq(FFT에서 가장 큰 단일 bin의 위치)만 특징으로 썼는데,
        FFT 해상도 한계 때문에 이 값의 변동성이 거의 0으로 나와 판별에 기여를
        못했다. 대신 2차/3차 배음 "대역 에너지"를 특징으로 추가해서, 고장 시
        3차 배음이 커지는 패턴을 더 안정적으로 잡아낸다.
      - rotation: 회전수 기준축 상태를 들고 있는 RotationTracker 인스턴스.
        연속된 스트림(같은 설비를 실시간으로 관측하는 흐름)에서는 호출자가
        트래커를 하나 만들어 매 윈도우 같은 객체를 넘겨야 점프 필터와
        re-lock이 동작한다. 서로 무관한 독립 샘플에는 None을 넘기면
        탐색범위 내 피크를 그대로 채택한다(상태 없는 단발 추정).

    반환하는 특징:
        rms: 신호 유효값 (에너지 크기, 고장 시 대체로 증가)
        kurtosis_val: 첨도 (신호가 얼마나 뾰족한지, 베어링 결함 시 증가하는 경향)
        peak_freq: FFT에서 가장 큰 피크가 나타난 주파수 (참고용, 단독 판별엔 부적합)
        peak_magnitude: 그 피크의 크기
        rotation_hz_estimate: 이번 윈도우에서 추정/갱신된 회전 주파수 (다음 호출에 체이닝)
        harmonic1_energy: rotation_hz_estimate x1 대역 에너지 - 불평형 고장의 직접 지표
        harmonic2_energy: rotation_hz_estimate x2 대역 에너지 - 정렬 불량의 직접 지표
        harmonic3_energy: rotation_hz_estimate x3 대역 에너지 - 이완 고장의 지표
        high_freq_energy: 250~400Hz 대역 에너지 - 시뮬레이션에서 저주파 고장에
            동반되는 고주파 부가 성분을 잡기 위한 보조 지표. 실제 베어링 결함
            진단(1~20kHz + 포락선 분석)과는 다른 개념이며, 본 프로젝트는
            베어링 정밀진단을 범위 밖으로 명시적으로 제외함.
    """
    rms = float(np.sqrt(np.mean(signal ** 2)))
    kurtosis_val = float(kurtosis(signal))  # 정규분포 기준 0 (scipy는 excess kurtosis 반환)

    freqs, magnitudes = compute_fft(signal, sample_rate)

    # DC 성분(0Hz)은 회전 신호와 무관하므로 제외하고 피크 탐색
    valid_idx = freqs > 1.0
    peak_idx = np.argmax(magnitudes[valid_idx])
    peak_freq = float(freqs[valid_idx][peak_idx])
    peak_magnitude = float(magnitudes[valid_idx][peak_idx])

    candidate_hz = estimate_rotation_hz(freqs, magnitudes)
    rotation_hz_estimate = (candidate_hz if rotation is None
                            else rotation.accept(candidate_hz))
    # 탐색범위 내 피크가 없으면 None이 될 수 있다(현재 윈도우/샘플레이트에선
    # 발생 안 하지만 방어적으로 처리). 이 경우 정격 회전주파수(FAN_ROTATION_HZ)로
    # 폴백해 하모닉 밴드 계산이 깨지지 않게 한다.
    if rotation_hz_estimate is None:
        rotation_hz_estimate = gen.FAN_ROTATION_HZ
    base_hz = rotation_hz_estimate
    harmonic1_energy = frequency_band_energy(freqs, magnitudes, base_hz * 1)
    harmonic2_energy = frequency_band_energy(freqs, magnitudes, base_hz * 2)
    harmonic3_energy = frequency_band_energy(freqs, magnitudes, base_hz * 3)
    high_freq_energy = float(np.sum(magnitudes[(freqs >= 250) & (freqs <= 400)]))

    return {
        "rms": rms,
        "kurtosis": kurtosis_val,
        "peak_freq": peak_freq,
        "peak_magnitude": peak_magnitude,
        "rotation_hz_estimate": base_hz,
        "harmonic1_energy": harmonic1_energy,
        "harmonic2_energy": harmonic2_energy,
        "harmonic3_energy": harmonic3_energy,
        "high_freq_energy": high_freq_energy,
    }


if __name__ == "__main__":
    # 01번 신호 생성기로 정상/고장 신호를 만들어 특징값 비교
    t_n, sig_n = gen.generate_normal_signal(seed=1)
    t_f, sig_f = gen.generate_faulty_signal(seed=1)

    # WINDOW_SIZE(256)만큼만 잘라서 사용 (실제 ESP32에서도 윈도우 단위로 처리)
    features_normal = extract_features(sig_n[:WINDOW_SIZE])
    features_faulty = extract_features(sig_f[:WINDOW_SIZE])

    print("=== 특징 추출 비교 (정상 vs 고장) ===")
    print(f"{'특징':<15}{'정상':>12}{'고장':>12}{'변화':>12}")
    for key in features_normal:
        n_val = features_normal[key]
        f_val = features_faulty[key]
        change = f_val - n_val
        print(f"{key:<15}{n_val:>12.4f}{f_val:>12.4f}{change:>+12.4f}")

    print("\n다음 단계: baseline_detector.py 에서 이 특징값으로 판별 로직 검증")
