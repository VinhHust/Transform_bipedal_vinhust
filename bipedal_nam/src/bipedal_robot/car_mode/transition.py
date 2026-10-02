# CÁI NÀY DÙNG ĐỂ ĐỔI MODE, KHI KHỞI ĐỘNG ĐỌC SERVO POSI => ĐANG NẰM Ở VỊ TRÍ NÀO

from .mode_controller import RobotMode

BUB_ID = 4


# vùng giao của bub giữa car và bipedal cho chuyển mode
def transition_range(limits_bipedal: dict, limits_car: dict) -> tuple:
    """Vùng giao của bub giữa 2 bảng. Tính từ bảng -> đo lại bảng thì vùng tự đổi theo."""
    lo = max(limits_bipedal[BUB_ID]["min"], limits_car[BUB_ID]["min"])
    hi = min(limits_bipedal[BUB_ID]["max"], limits_car[BUB_ID]["max"])
    if lo > hi:
        raise ValueError(f"Bub 2 bảng không giao nhau ({lo} > {hi}) -> không bao giờ đổi được mode")
    return lo, hi


def check_mode_change(current: RobotMode, requested, bub: int, lo: int, hi: int):
    if str(requested).upper() == current.value:
        return None
    if lo <= bub <= hi:
        return None
    return f"bub={bub} ngoài vùng chuyển {lo}–{hi}: đưa tay về transition pose trước"


# mode lúc server khởi động -> dựa trên bub đo được
def mode_from_bub(bub: int, lo: int, hi: int) -> RobotMode:
    if bub < lo:
        return RobotMode.BIPEDAL
    return RobotMode.CAR  # trong vùng giao thì mode nào cũng an toàn -> chọn CAR
