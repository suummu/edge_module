"""
RotationTracker 회귀 테스트 — 기준축 영구 락 결함 방지.

[무엇을 막는 테스트인가]
회전수 기준축은 max_jump_hz를 넘는 후보를 노이즈로 보고 거부한다. 그런데
거부 상태를 해제할 조건이 없으면, 가변전압/VFD로 회전수를 한 번에 크게
바꿨을 때 이후 모든 후보가 계속 거부되어 기준축이 옛 값에 영구 고정된다.
그러면 하모닉 대역이 엉뚱한 중심에 놓여 harmonic1/2/3_energy가 전부
무의미해진다. 팬 테스트베드는 가변 전압 공급기로 회전수를 조절하므로
이 경로는 실제 시연에서 그대로 발생한다.

기존 테스트가 이 결함을 못 잡은 이유: "회전수 ±8% 드리프트 강건성"만
검사했는데, 8%는 max_jump_hz(5Hz)보다 작고 점진적이라 거부 자체가
발생하지 않는다. 계단형(step) 변화 케이스가 아예 없었다.

실행: python -m tests.test_rotation_tracker   (또는 pytest)
"""
import numpy as np

from src.feature_extraction import RotationTracker, extract_features
from src.signal_generator import SAMPLE_RATE, WINDOW_SIZE


def test_tracks_small_changes_immediately():
    """max_jump_hz 이내의 변화는 즉시 추종한다."""
    rt = RotationTracker(max_jump_hz=5.0)
    assert rt.accept(50.0) == 50.0          # 최초 추정은 무조건 채택
    assert rt.accept(52.0) == 52.0
    assert rt.accept(49.5) == 49.5
    assert rt.rejected == 0


def test_rejects_scattered_noise():
    """매번 다른 곳에 튀는 후보는 거부하고 이전 값을 유지한다."""
    rt = RotationTracker(max_jump_hz=5.0, relock_windows=5)
    rt.accept(50.0)
    for spike in (68.0, 31.0, 67.0, 30.0, 69.0, 32.0, 66.0):
        rt.accept(spike)
        assert rt.hz == 50.0, "산발적 노이즈에 기준축이 끌려갔다"


def test_relocks_on_sustained_speed_change():
    """★ 핵심 회귀 — 지속되는 계단형 변화는 새 운전점으로 수용해야 한다."""
    rt = RotationTracker(max_jump_hz=5.0, relock_windows=5)
    rt.accept(50.0)

    for i in range(4):                       # relock 직전까지는 유지
        rt.accept(62.0)
        assert rt.hz == 50.0, f"{i + 1}번째에 너무 일찍 수용됐다"

    rt.accept(62.0)                          # 5번째 = relock_windows 도달
    assert rt.hz == 62.0, "지속된 속도 변화를 영구 거부했다 (기준축 락)"
    assert rt.rejected == 0
    assert rt.pending_hz is None

    rt.accept(62.5)                          # 새 운전점 기준으로 정상 추종
    assert rt.hz == 62.5


def test_normal_tracking_resets_reject_count():
    """정상 추종이 한 번 들어오면 누적 거부가 해제된다 (단조 증가 카운터 아님)."""
    rt = RotationTracker(max_jump_hz=5.0, relock_windows=5)
    rt.accept(50.0)
    rt.accept(62.0)
    rt.accept(62.0)
    assert rt.rejected == 2
    rt.accept(51.0)                          # 원래 자리 근처로 복귀
    assert rt.rejected == 0 and rt.hz == 51.0

    for _ in range(4):                       # 다시 밀어도 카운트는 처음부터
        rt.accept(62.0)
        assert rt.hz == 51.0


def test_none_candidate_keeps_last_estimate():
    """탐색범위에 피크가 없어 후보가 없으면 마지막 추정치를 유지한다."""
    rt = RotationTracker()
    assert rt.accept(None) is None           # 최초에 후보가 없으면 미확정
    rt.accept(50.0)
    assert rt.accept(None) == 50.0


def _sine_window(hz, seed):
    """지정한 회전 주파수의 1윈도우 신호 (기본파 + 2차 배음 + 노이즈)."""
    rng = np.random.default_rng(seed)
    t = np.arange(WINDOW_SIZE) / SAMPLE_RATE
    sig = np.sin(2 * np.pi * hz * t) + 0.15 * np.sin(2 * np.pi * 2 * hz * t)
    return sig + rng.normal(0, 0.05, size=t.shape)


def test_end_to_end_step_change_through_extract_features():
    """실제 신호 스트림에서 계단형 속도 변화를 따라잡는지 확인."""
    rt = RotationTracker(max_jump_hz=5.0, relock_windows=5)

    for s in range(10):                      # 50Hz 정상 운전
        extract_features(_sine_window(50.0, seed=s), rotation=rt)
    assert abs(rt.hz - 50.0) < 4.0, f"정상 구간 추정 실패: {rt.hz}"

    locked_at = rt.hz
    extract_features(_sine_window(65.0, seed=100), rotation=rt)
    assert rt.hz == locked_at, "단발 변화를 즉시 수용해버렸다"

    for s in range(101, 110):                # 65Hz로 계속 운전
        extract_features(_sine_window(65.0, seed=s), rotation=rt)
    assert abs(rt.hz - 65.0) < 4.0, f"속도 변경을 못 따라잡았다: {rt.hz}"


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {name}: {e}")
    print("-" * 50)
    print("모두 통과" if failed == 0 else f"{failed}건 실패")
    raise SystemExit(1 if failed else 0)
