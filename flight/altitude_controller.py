from config import ALTITUDE_TOLERANCE_M
from flight.navigation import Navigation


class AltitudeController:
    def __init__(self, navigation: Navigation):
        self.navigation = navigation

    def goto(self, altitude_m, timeout=30.0):
        self.navigation.set_relative_altitude(altitude_m)
        return self.navigation.wait_until_altitude(
            altitude_m,
            ALTITUDE_TOLERANCE_M,
            timeout,
        )
