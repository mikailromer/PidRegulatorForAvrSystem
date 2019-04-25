import xml.etree.ElementTree as ElementTree
from os import path, getcwd

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
                if systemElement.tag == "Amplifier":
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
        FaAlgorithm= {"N":NumberOfFireflies, "T":MaxGenerations, "beta0": InitialAttractiveness,\
                      "gamma": AbsorptionCoefficient, "delta": WeightingFactor, "step": Step }

    return FaAlgorithm, amplifier, exciter, generator, sensor

if __name__ == '__main__':

    FaAlgorithm, amp, ex, gen, sen =parseConfigurationFile()
    print("DEBUG")
