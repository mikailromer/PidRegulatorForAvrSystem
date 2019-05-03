import xml.etree.ElementTree as ElementTree
import matplotlib.pyplot as plt
from control import tf,step_response
from os import path, getcwd, mkdir
import sys
from FaAlgorithmFiles.FaAlgorithmFunctions import *
import matplotlib.pyplot as plt
import numpy as np


def parseConfigurationFile():
    filePath=path.join(getcwd(), "config", "config.xml")
    xmlTree=ElementTree.parse(filePath)
    root=xmlTree.getroot()
    for rootChild in root:
        if rootChild.tag == "FaAlgorithmParameters":
            for parameter in rootChild.findall("Parameter"):
                if parameter.get("name") == "NumberOfFireflies":
                    NumberOfFireflies = int(parameter.get("value"))
                elif parameter.get("name") == "MaxGenerations":
                    MaxGenerations = int(parameter.get("value"))
                elif parameter.get("name") == "InitialAttractiveness":
                    InitialAttractiveness = float(parameter.get("value"))
                elif parameter.get("name") == "AbsorptionCoefficient":
                    AbsorptionCoefficient = float(parameter.get("value"))
                elif parameter.get("name") == "Randomness":
                    Randomness = float(parameter.get("value"))
                elif parameter.get("name") == "WeightingFactor":
                    WeightingFactor = float(parameter.get("value"))
                elif parameter.get("name") == "FitnessFunctionCoefficient":
                    FitnessFunctionCoefficient = float(parameter.get("value"))
                elif parameter.get("name") == "Step":
                    Step = float(parameter.get("value"))

        elif rootChild.tag == "AVR":
            for systemElement in rootChild:
                if systemElement.tag == "VoltageReference":
                    voltageReference = float(systemElement.get("value"))

                elif systemElement.tag == "Amplifier":
                    Ka = float(systemElement.get("Gain"))
                    Ta = float(systemElement.get("ConstantTime"))
                    amplifier = {"Ka": Ka , "Ta": Ta}

                elif systemElement.tag == "Exciter":
                    Ke = float(systemElement.get("Gain"))
                    Te = float(systemElement.get("ConstantTime"))
                    exciter = {"Ke": Ke, "Te": Te}

                elif systemElement.tag == "Generator":
                    Kg = float(systemElement.get("Gain"))
                    Tg = float(systemElement.get("ConstantTime"))
                    generator = {"Kg": Kg, "Tg": Tg}

                elif systemElement.tag == "Sensor":
                    Ks = float(systemElement.get("Gain"))
                    Ts = float(systemElement.get("ConstantTime"))
                    sensor = {"Ks": Ks, "Ts": Ts}

        elif rootChild.tag == "PID":
            for PidElement in rootChild:
                if PidElement.get("name") == "Proportional":
                    maxKp = float(PidElement.get("maxValue"))

                elif PidElement.get("name") == "Integral":
                    maxKi = float(PidElement.get("maxValue"))

                elif PidElement.get("name") == "Derivative":
                    maxKd = float(PidElement.get("maxValue"))

            PID = {"Kp": maxKp, "Ki": maxKi, "Kd": maxKd}

        FaAlgorithm= {"N":NumberOfFireflies, "T":MaxGenerations, "beta0": InitialAttractiveness,"alfa": Randomness,\
                      "gamma": AbsorptionCoefficient, "delta": WeightingFactor, "ro": FitnessFunctionCoefficient, "step": Step }

    return FaAlgorithm, amplifier, exciter, generator, sensor, voltageReference, PID

def step_info(t,Vout):
    overshoot =(Vout.max()/Vout[-1]-1)
    risingTime = t[next(i for i in range(0,len(Vout)-1) if Vout[i]>Vout[-1]*.90)]-t[0]
    settingTime = t[next(len(Vout)-i for i in range(2,len(Vout)-1) if abs(Vout[-i]/Vout[-1])>1.02)]-t[0]
    print("OS: %f%s"%(overshoot,'%'))
    print("Tr: %fs"%(risingTime))
    print("Ts: %fs"%(settingTime))

    return overshoot, risingTime, settingTime

if __name__ == '__main__':

    faAlgorithmParams, amplifierParams, exciterParams,\
    generatorParams, sensorParams, voltageReference, pidGainsThresholds =parseConfigurationFile()

    # Kp = 1.3
    # Ki= 0.7
    # Kd = 0.1
    # Laplace's transform for elements of AVR system
#    Gpid = Kp + tf([Ki],[1,0]) + tf([Kd,0],[1])
    Ga = tf([amplifierParams["Ka"]], [amplifierParams["Ta"], 1])
    Ge = tf([exciterParams["Ke"]], [exciterParams["Te"], 1])
    Gg = tf([generatorParams["Kg"]], [generatorParams["Tg"], 1])
    Gs = tf([sensorParams["Ks"]], [sensorParams["Ts"], 1])
    tableOfPoints=[]
    collectDictionaryOfBestGains = np.array([])
    bestFirefly = None
    swarmOfFireflies = createSwarmOfFireflies(faAlgorithmParams["N"], pidGainsThresholds, faAlgorithmParams["beta0"])
    for t in range(faAlgorithmParams["T"]):
        for i in range(len(swarmOfFireflies)):
            for j in range(len(swarmOfFireflies)):
                calculateFireflyLigthIntensivity(swarmOfFireflies[i], Ga, Ge, Gg, Gs, voltageReference, faAlgorithmParams["ro"])
                calculateFireflyLigthIntensivity(swarmOfFireflies[j], Ga, Ge, Gg, Gs, voltageReference, faAlgorithmParams["ro"])
                if swarmOfFireflies[j].getLigthIntensivityValue() > swarmOfFireflies[i].getLigthIntensivityValue():
                    Rij = ComputeDistanceBeetweenTwoObjects(swarmOfFireflies[i],swarmOfFireflies[j])
#                    beta = AtractivenessFunction(faAlgorithmParams["beta0"],Rij,faAlgorithmParams["gamma"])
                    swarmOfFireflies[i].set_beta(AtractivenessFunction(faAlgorithmParams["beta0"],Rij,faAlgorithmParams["gamma"]))
                    u = GenerateRandomVector()
                    KPi = swarmOfFireflies[i].get_Kp() + swarmOfFireflies[i].get_beta() *(swarmOfFireflies[j].get_Kp()\
                                                                - swarmOfFireflies[i].get_Kp()) + faAlgorithmParams["alfa"]*u["Kp"]
                    KIi = swarmOfFireflies[i].get_Ki() + swarmOfFireflies[i].get_beta() * (swarmOfFireflies[j].get_Ki() \
                                                                 - swarmOfFireflies[i].get_Ki()) + faAlgorithmParams["alfa"] * u["Ki"]
                    KDi = swarmOfFireflies[i].get_Kd() + swarmOfFireflies[i].get_beta() * (swarmOfFireflies[j].get_Kd() \
                                                                 - swarmOfFireflies[i].get_Kd()) + faAlgorithmParams["alfa"] * u["Kd"]
                    swarmOfFireflies[i].set_PidGains(KPi, KIi, KDi)
                    Rij = ComputeDistanceBeetweenTwoObjects(swarmOfFireflies[i], swarmOfFireflies[j])
                   # beta = AtractivenessFunction(faAlgorithmParams["beta0"], Rij, faAlgorithmParams["gamma"])
                    swarmOfFireflies[i].set_beta(AtractivenessFunction(faAlgorithmParams["beta0"], Rij, faAlgorithmParams["gamma"]))

        uk = GenerateRandomVector()
        IndexOfTheMostAtractiveFirefly = FindTheMostAtractiveFirefly(swarmOfFireflies)
        TheMostAtractiveFirefly_Kp = swarmOfFireflies[IndexOfTheMostAtractiveFirefly].get_Kp() + uk["Kp"]
        TheMostAtractiveFirefly_Ki = swarmOfFireflies[IndexOfTheMostAtractiveFirefly].get_Ki() + uk["Ki"]
        TheMostAtractiveFirefly_Kd = swarmOfFireflies[IndexOfTheMostAtractiveFirefly].get_Kd() + uk["Kd"]
        swarmOfFireflies[IndexOfTheMostAtractiveFirefly].set_PidGains(TheMostAtractiveFirefly_Kp,
                                                                   TheMostAtractiveFirefly_Ki, TheMostAtractiveFirefly_Kd )
        calculateFireflyLigthIntensivity(swarmOfFireflies[IndexOfTheMostAtractiveFirefly],Ga,Ge,Gg, Gs, voltageReference,faAlgorithmParams["ro"])
        Best = swarmOfFireflies[FindTheMostAtractiveFirefly(swarmOfFireflies)]
        collectDictionaryOfBestGains = np.append(collectDictionaryOfBestGains, Best.get_PidGains())
        #tableOfPoints.append(collectListOfPoints(swarmOfFireflies))
        # sys.stdout.write("\rGeneration:%4d, BestFitness:%.10f" % (
        #     t, swarmOfFireflies[IndexOfTheMostAtractiveFirefly].getLigthIntensivityValue()))

        print('\n')





    plt.plot(Best.getResponseTimeVector(),Best.getResponseOutputVoltageVector())
    plt.title("AVR Regulation System's Response")
    plt.xlabel("Time [s]")
    plt.xlabel("Output Voltage [V]")
    plt.xlim((0,Best.getResponseTimeVector()[len(Best.getResponseTimeVector())-1]))
    plt.grid()
    plt.show()



