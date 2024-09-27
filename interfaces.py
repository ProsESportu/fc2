from abc import ABCMeta, abstractmethod


class IDrives(metaclass=ABCMeta):
    @abstractmethod
    def set(self, yaw: float, pitch: float, roll: float, throttle: float):
        pass


class IRCReceiver(metaclass=ABCMeta):
    @abstractmethod
    def get_mapped(self) -> list[float]:
        """
        Returns the current values of the sliders as a list of floats.
        @return: A list of floats representing the current values of the sliders.[yaw, pitch, roll, throttle]
        """
        pass


class IGyro(metaclass=ABCMeta):
    @abstractmethod
    def read_calibrated_mapped(self) -> tuple[float, float, float]:
        """
        Reads the raw values from the gyro and returns them as a tuple of floats.
        @return: A tuple of floats representing the x, y, and z values of the gyro.
        """
        pass
