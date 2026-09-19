"""旧名の互換入口。新規featureではRunByGyro(target=profile.heading_at)を使う。"""

from .gyro_drive import RunByGyro
from ..heading_profile import HeadingProfile
from ..types import HeadingType


class RunByDistanceHeading(RunByGyro):
    """既存呼出しのための変換だけを行う。制御処理はRunByGyroに集約。"""

    def __init__(self, name, points, power=33, pid_p=1.1, pid_i=0.1, pid_d=0.03,
                 completion_condition=None, completion_min_mm=0):
        self.profile = HeadingProfile(points)
        super().__init__(
            name=name, target=self.profile.heading_at, power=power,
            pid_p=pid_p, pid_i=pid_i, pid_d=pid_d,
            target_type=HeadingType.RELATIVE,
            distance_limit_mm=self.profile.length_mm,
            completion_condition=completion_condition,
            completion_min_mm=completion_min_mm,
        )
