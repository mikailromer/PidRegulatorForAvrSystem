from control import tf
import numpy as np

class Firefly(object):
    def __init__(self, Kp, Ki, Kd, pidGainsThresholds, beta0):
        self.__PidGains = {"Kp": Kp, "Ki": Ki, "Kd": Kd}
        self.__pidGainsThresholds = pidGainsThresholds
        self.__atractiveness = None
        self.__fitnessFunctionValue = None
        self.__ligthIntensivity = None
        self.__beta = beta0
        self.__PidControllerTransferFunction = self.get_Kp() + tf([self.get_Ki()],[1,0]) + tf([self.get_Kd(),0],[1])
        self.__responseTimeVector = None
        self.__responseOutputVoltageVector = None
        self.__Ess = None
        self.__M = None
        self.__ITAE = None
        self.__Ts = None
        self.__Tr = None

    def get_Kp(self):
        return self.__PidGains["Kp"]

    def get_Ki(self):
        return self.__PidGains["Ki"]

    def get_Kd(self):
        return self.__PidGains["Kd"]

    def get_PidGains(self):
        return self.__PidGains

    def getFitnessFunctionValue(self):
        return self.__fitnessFunctionValue

    def getLigthIntensivityValue(self):
        return self.__ligthIntensivity

    def getPidControllerTransferFunction(self):
        return self.__PidControllerTransferFunction

    def getEss(self):
        return self.__Ess

    def getTs(self):
        return self.__Ts

    def getTr(self):
        return self.__Tr

    def getM(self):
        return self.__M

    def get_beta(self):
        return self.__beta

    def getResponseTimeVector(self):
        return self.__responseTimeVector

    def getResponseOutputVoltageVector(self):
        return self.__responseOutputVoltageVector

    def getPidGainsThresholds(self):
        return self.__pidGainsThresholds

    def setFitnessFunctionValue(self, value):
        self.__fitnessFunctionValue = value

    def setLigthIntensivity(self):
        self.__ligthIntensivity = self.ligthIntensivity(self.__fitnessFunctionValue)

    def set_beta(self,beta):
        self.__beta=beta

    def set_PidGains(self, Kp, Ki, Kd):

        self.__PidGains = {"Kp": Kp, "Ki": Ki, "Kd": Kd}

        if self.get_Kp() > self.__pidGainsThresholds["Kp"]:

            self.__PidGains = {"Kp": self.__pidGainsThresholds["Kp"], "Ki": Ki, "Kd": Kd}

        if self.get_Kp() < 0:
            self.__PidGains = {"Kp": 0, "Ki": Ki, "Kd": Kd}

        if self.get_Ki() > self.__pidGainsThresholds["Ki"]:
            self.__PidGains = {"Kp": Kp, "Ki": self.__pidGainsThresholds["Ki"], "Kd": Kd}

        if self.get_Ki() < 0:
            self.__PidGains = {"Kp": Kp, "Ki": 0, "Kd": Kd}

        if self.get_Kd() > self.__pidGainsThresholds["Kd"]:
            self.__PidGains = {"Kp": Kp, "Ki": Ki, "Kd": self.__pidGainsThresholds["Kd"]}

        if self.get_Ki() < 0:
            self.__PidGains = {"Kp": Kp, "Ki": Ki, "Kd": 0}



    def setPidControllerTransferFunction(self, Kp, Ki, Kd):
        self.__PidControllerTransferFunction = Kp+ tf([Ki], [1, 0]) + tf([Kd, 0], [1])

    def setAvrSystemResponseParameters(self, Ess, M, Tr, Ts, responseTimeVector, responseOutputVoltageVector):
        self.__Ess = Ess
        self.__M = M
        self.__Tr = Tr
        self.__Ts = Ts
        self.__responseTimeVector = responseTimeVector
        self.__responseOutputVoltageVector= responseOutputVoltageVector

    def ligthIntensivity(self, fitnessFunctionValue):
        return np.exp(-fitnessFunctionValue)




