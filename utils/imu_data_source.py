"""
IMU 数据源抽象接口
==================
定义统一的数据源接口，所有接收器（BLE、WiFi、文件）都实现它。
这样上层控制逻辑无需关心数据来源。
"""

from abc import ABC, abstractmethod
from scipy.spatial.transform import Rotation as R
from typing import Optional


class IMUDataSource(ABC):
    """IMU 数据源的抽象基类"""

    @abstractmethod
    def get_orientation(self) -> Optional[R]:
        """
        获取当前姿态（已校准后的相对旋转）

        Returns:
            Rotation: Scipy Rotation 对象，表示相对于校准零位的旋转
            None: 如果当前无数据
        """
        pass

    @abstractmethod
    def calibrate(self):
        """
        校准：将当前姿态设为零位
        调用后，get_orientation() 返回的是相对于此刻的增量旋转
        """
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """检查数据源是否已连接/可用"""
        pass

    def get_euler_degrees(self) -> Optional[tuple]:
        """
        便捷方法：获取欧拉角 (roll, pitch, yaw) 单位为度
        """
        r = self.get_orientation()
        if r is None:
            return None
        return tuple(r.as_euler('xyz', degrees=True))
