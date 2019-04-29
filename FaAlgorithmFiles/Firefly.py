#from CommonFunctions import CostFunction

class Firefly(object):
    def __init__(self, Kp, Ki,Kd, beta0):
        self.__PidGains = {"Kp": Kp, "Ki": Ki, "Kd": Kd}
        self.__atractiveness = None
        self.__fitnessFunctionValue = None
        self.__beta = beta0

    def get_Kp(self):
        return self.__PidGains["Kp"]

    def get_Ki(self):
        return self.__PidGains["Ki"]

    def get_Kd(self):
        return self.__PidGains["Kd"]

    def get_PidGains(self):
        return self.__PidGains

    def getFitnessFunctionValue(self, value):
        return self.__fitnessFunctionValue

    def setFitnessFunctionValue(self, value):
        self.__fitnessFunctionValue = value

    # def set_PidGains(self, Kp, Ki, Kd):
    #     self.__PidGains = {"Kp": Kp, "Ki": Ki, "Kd": Kd}
    #
    #     if self.get_X() > Xmax:
    #         self.__Point = {"X": Xmax, "Y": Y, "Z": CostFunction(Xmax, Y)}
    #
    #     if self.get_X() < Xmin:
    #         self.__Point = {"X": Xmin, "Y": Y, "Z": CostFunction(Xmin, Y)}
    #
    #     if self.get_Y() > Ymax:
    #         self.__Point = {"X": X, "Y": Ymax, "Z": CostFunction(X, Ymax)}
    #
    #     if self.get_Y() < Ymin:
    #         self.__Point = {"X": X, "Y": Ymin, "Z": CostFunction(X, Ymin)}

    def get_beta(self):
        return self.__beta

    def set_beta(self,beta):
        self.__beta=beta


