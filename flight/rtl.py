from flight.navigation import Navigation


class RTLController:
    def __init__(self, navigation: Navigation):
        self.navigation = navigation

    def start(self):
        self.navigation.rtl()

    def wait_for_completion(self, timeout):
        return self.navigation.wait_for_rtl_completion(timeout)
