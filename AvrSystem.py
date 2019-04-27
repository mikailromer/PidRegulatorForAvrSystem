import xml.etree.ElementTree as ElementTree
import matplotlib.pyplot as plt
from control import tf,step_response
from os import path, getcwd, mkdir
from FaAlgorithmFiles.FaAlgorithmFunctions import *


def parseConfigurationFile():
    filePath=path.join(getcwd(), "config", "config.xml")
    xmlTree=ElementTree.parse(filePath)
    root=xmlTree.getroot()
    for rootChild in root:
        if rootChild.tag == "FaAlgorithmParameters":
            for parameter in rootChild.findall("Parameter"):
                if parameter.get("name") == "NumberOfFireflies":
                    NumberOfFireflies = parameter.get("value")
                elif parameter.get("name") == "MaxGenerations":
                    MaxGenerations = parameter.get("value")
                elif parameter.get("name") == "InitialAttractiveness":
                    InitialAttractiveness = parameter.get("value")
                elif parameter.get("name") == "AbsorptionCoefficient":
                    AbsorptionCoefficient = parameter.get("value")
                elif parameter.get("name") == "Randomness":
                    Randomness = parameter.get("value")
                elif parameter.get("name") == "WeightingFactor":
                    WeightingFactor = parameter.get("value")
                elif parameter.get("name") == "Step":
                    Step = parameter.get("value")

        elif rootChild.tag == "AVR":
            for systemElement in rootChild:
                if systemElement.tag == "VoltageReference":
                    voltageReference = systemElement.get("VoltageReference")

                elif systemElement.tag == "Amplifier":
                    Ka = systemElement.get("Gain")
                    Ta = systemElement.get("ConstantTime")
                    amplifier = {"Ka": Ka , "Ta": Ta}

                elif systemElement.tag == "Exciter":
                    Ke = systemElement.get("Gain")
                    Te = systemElement.get("ConstantTime")
                    exciter = {"Ke": Ke, "Te": Te}

                elif systemElement.tag == "Generator":
                    Kg = systemElement.get("Gain")
                    Tg = systemElement.get("ConstantTime")
                    generator = {"Kg": Kg, "Tg": Tg}

                elif systemElement.tag == "Sensor":
                    Ks = systemElement.get("Gain")
                    Ts = systemElement.get("ConstantTime")
                    sensor = {"Ks": Ks, "Ts": Ts}

        elif rootChild.tag == "PID":
            for PidElement in rootChild:
                if PidElement.get("name") == "Proportional":
                    maxKp = systemElement.get("maxValue")

                elif PidElement.get("name") == "Integral":
                    maxKi = systemElement.get("maxValue")

                elif PidElement.get("name") == "Derivative":
                    maxKd = systemElement.get("maxValue")

            PID = {"Kp": maxKp, "Ki": maxKi, "Kd": maxKd}

        FaAlgorithm= {"N":NumberOfFireflies, "T":MaxGenerations, "beta0": InitialAttractiveness,\
                      "gamma": AbsorptionCoefficient, "delta": WeightingFactor, "step": Step }

    return FaAlgorithm, amplifier, exciter, generator, sensor, voltageReference, PID

def step_info(t,Vout):
    overshoot =(Vout.max()/Vout[-1]-1)*100
    risingTime = t[next(i for i in range(0,len(Vout)-1) if Vout[i]>Vout[-1]*.90)]-t[0]
    settingTime = t[next(len(Vout)-i for i in range(2,len(Vout)-1) if abs(Vout[-i]/Vout[-1])>1.02)]-t[0]
    print("OS: %f%s"%(overshoot,'%'))
    print("Tr: %fs"%(risingTime))
    print("Ts: %fs"%(settingTime))

    return overshoot, risingTime, settingTime



if __name__ == '__main__':

    faAlgorithmParams, amplifierParams, exciterParams,\
    generatorParams, sensorParams, voltageReference, pidGainsThresholds =parseConfigurationFile()

    Kp = 1.3
    Ki= 0.7
    Kd = 0.1
    # Laplace's transform for elements of AVR system
    Gpid = Kp + tf([Ki],[1,0]) + tf([Kd,0],[1])
    Ga = tf([float(amplifierParams["Ka"])], [float(amplifierParams["Ta"]), 1])
    Ge = tf([float(exciterParams["Ke"])], [float(exciterParams["Te"]), 1])
    Gg = tf([float(generatorParams["Kg"])], [float(generatorParams["Tg"]), 1])
    Gs = tf([float(sensorParams["Ks"])], [float(sensorParams["Ts"]), 1])

    swarmOfFireflies = createSwarmOfFireflies(faAlgorithmParams["N"], pidGainsThresholds, faAlgorithmParams["beta0"])
    for t in range(faAlgorithmParams["T"]):
        for firefly in swarmOfFireflies:
            Gpid = firefly.get_Kp() + tf([firefly.get_Ki()],[1,0]) + tf([firefly.get_Kd(),0],[1])
            systemTransferFunction = (Gpid * Ga * Ge * Gg) / (1 + (Gpid * Ga * Ge * Gg))
            T, Vout = step_response(systemTransferFunction * voltageReference)
            M, Tr, Ts = step_info(T, Vout)



    systemTransferFunction= (Gpid *Ga * Ge * Gg)/(1 + (Gpid *Ga * Ge * Gg))
    T, Vout= step_response(systemTransferFunction*voltageReference)
    M, Tr, Ts = step_info(T,Vout)
    plt.plot(T,Vout)
    plt.title("AVR Regulation System's Response")
    plt.xlabel("Time [s]")
    plt.xlabel("Output Voltage [V]")
    plt.xlim((0,T[len(T)-1]))
    plt.grid()
    plt.show()



