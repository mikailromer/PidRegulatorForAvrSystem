import numpy as np
from FaAlgorithmFiles.Firefly import Firefly
from control import step_response
from control import pole as Pole
import matplotlib.pyplot as plt


deltaT =0.1


def createSwarmOfFireflies(NumberOfFireflies, pidGainsThresholds, beta0):
    SwarmOfFireflies = []
    for index in range(NumberOfFireflies):
        Kp = round(np.random.uniform(0, pidGainsThresholds["Kp"]), 3)
        Ki = round(np.random.uniform(0, pidGainsThresholds["Ki"]), 3)
        Kd = round(np.random.uniform(0, pidGainsThresholds["Kd"]), 3)
        SwarmOfFireflies.append(Firefly(Kp, Ki, Kd,pidGainsThresholds, beta0))
    return SwarmOfFireflies

def interpolationOfOutputVoltage(T, Vout):
    interpTime = np.array([])
    t = 0
    while t < T[len(T) - 1]:
        interpTime = np.append(interpTime, t)
        t = t + deltaT

    interpVout = np.interp(interpTime, T, Vout)
    return interpTime, interpVout

def checkAvrSystemStability(firefly):

    poles = Pole(firefly.getAvrSystemTransferFunction())
    stabilityCondition = True
    for pole in poles:
        if (pole.real > 0):
            stabilityCondition = False
            break

    return stabilityCondition

def makeAvrSystemStable(Ga,Ge,Gg,Gs, firefly):
    stabilityCondition = checkAvrSystemStability(firefly)
    while stabilityCondition==False:
        Kp = round(np.random.uniform(0, (firefly.getPidGainsThresholds())["Kp"]), 3)
        Ki = round(np.random.uniform(0, (firefly.getPidGainsThresholds())["Ki"]), 3)
        Kd = round(np.random.uniform(0, (firefly.getPidGainsThresholds())["Kd"]), 3)
        firefly.set_PidGains(Kp,Ki,Kd)
        firefly.setPidControllerTransferFunction(firefly.get_Kp(), firefly.get_Ki(), firefly.get_Kd())
        firefly.setAvrSystemTransferFunction(firefly.getPidControllerTransferFunction(), Ga, Ge, Gg, Gs)
        stabilityCondition = checkAvrSystemStability(firefly)

def ITAE(T, Vout, Vref):
    integral = 0
    errorVector = Vout - Vref
    for i in range(1,len(T)):
        if (errorVector[i] * errorVector[i-1])< 0:
            field= (deltaT/4)*(np.abs(errorVector[i-1]) + np.abs(errorVector[i]))
        else:
            field = (np.abs(errorVector[i] + errorVector[i-1]) * deltaT) /2

        integral = integral + field

    return integral


def fitnessFunction(T, Vout, M, Ess, Ts, Tr, ro, Vref):
    # elementI = ITAE(T, Vout, Vref)
    # elementII = (1 - np.exp(ro))*(M + Ess)
    # elementIII = np.exp(ro)*(Ts - Tr)
    # elementIIorazIII = np.abs((1 - np.exp(ro)) *(M + Ess) + np.exp(ro)*(Ts - Tr))
    fitnessFunctionValue = ITAE(T, Vout, Vref) *((1 - np.exp(ro)) *(M + Ess) + np.exp(ro)*(Ts - Tr))
    return fitnessFunctionValue

# def ligthIntensivity(fitnessFunctionValue):
#     return np.exp(-fitnessFunctionValue)

def step_info(t,Vout,firefly):
    #stability = checkAvrSystemStability(firefly)
    overshoot =(Vout.max()/Vout[-1]-1)
    risingTime = None
    # a =next(i for i in range(0,len(Vout)-1) if Vout[i]>Vout[-1]*.90)
    #risingTime = t[next(i for i in range(0,len(Vout)-1) if Vout[i]>Vout[-1]*.90)]-t[0]
    for i in range(0, len(Vout) -1):
        if abs(Vout[i]) > abs(Vout[-1] * .90):
            risingTime = t[i] -t[0]
            #risingTimeIndex = i
            break


    # if risingTime==None:[1,2,3,4,5,6]
    #     z,b = pzmap(firefly.getAvrSystemTransferFunction())
    #     plt.plot(t,Vout)
    #     plt.show()
    for i in range(2, len(Vout) - 1):
        if (abs((Vout[-i] - Vout[-1]) / Vout[-1]) > 0.02):
            settingTime = t[len(Vout)-i]-t[0]
            break

    # if stability == False:
    #     if risingTime == None:
    #         risingTime = t[len(Vout) -2] - t[0]
    #         print("DEBUG")

    #     if abs((Vout[-i] / Vout[-1])) > 1.02:
    #         condition = True
    #         settk = t[len(Vout)-i]-t[0]
    #         break
    #
    # if condition==False:
    #     print("Error")
    #settingTime = t[next(len(Vout)-i for i in range(2,len(Vout)-1) if abs(Vout[-i]/Vout[-1])>1.02)]-t[0]
    # if settingTime == None:
    #     raise Exception("No settiing time val")

    return overshoot, risingTime, settingTime

def calculateFireflyLigthIntensivity(firefly, Ga, Ge, Gg,Gs, voltageReference, ro):
    firefly.setPidControllerTransferFunction(firefly.get_Kp(), firefly.get_Ki(), firefly.get_Kd())
    firefly.setAvrSystemTransferFunction(firefly.getPidControllerTransferFunction(),Ga,Ge,Gg, Gs)
    makeAvrSystemStable(Ga,Ge,Gg,Gs,firefly)
    T, Vout = step_response(firefly.getAvrSystemTransferFunction() * voltageReference)
    interpTime, interpVout = interpolationOfOutputVoltage(T, Vout)
    Ess = np.abs(voltageReference - interpVout[len(interpVout) - 1])
    M, Tr, Ts = step_info(interpTime, interpVout, firefly)
    firefly.setAvrSystemResponseParameters(Ess, M, Tr, Ts, interpTime, interpVout)
    fitnessFunctionValue = fitnessFunction(interpTime,interpVout, M, Ess, Ts, Tr, ro,voltageReference)
    firefly.setFitnessFunctionValue(fitnessFunctionValue)
    firefly.setLigthIntensivity()

def ComputeDistanceBeetweenTwoObjects(Firefly_I, Firefly_J):
    if ((Firefly_I.get_Kp()==None and Firefly_I.get_Ki()==None and Firefly_I.get_Kd()==None)\
            or (Firefly_J.get_Kp()==None and Firefly_J.get_Ki()==None and Firefly_J.get_Kd()==None) ):
        return None
    else:
        deltaKp = Firefly_J.get_Kp() - Firefly_I.get_Kp()
        deltaKi = Firefly_J.get_Ki() - Firefly_I.get_Ki()
        deltaKd = Firefly_J.get_Kd() - Firefly_I.get_Kd()
        Rij = np.sqrt(deltaKp ** 2 + deltaKi ** 2 + deltaKd ** 2)
        return Rij

def AtractivenessFunction(beta0,Rij,Lambda):
    result=beta0*np.exp(-Lambda*(Rij**2))
    return result

def GenerateRandomVector(pidGainsThresholds):
    uKp = round(np.random.uniform(-pidGainsThresholds["Kp"]/100, pidGainsThresholds["Kp"]/100), 3)
    uKi = round(np.random.uniform(-pidGainsThresholds["Ki"]/100, pidGainsThresholds["Ki"]/100), 3)
    uKd = round(np.random.uniform(-pidGainsThresholds["Kd"]/100, pidGainsThresholds["Kd"]/100), 3)
    RandomVector = {"Kp": uKp, "Ki": uKi, "Kd":uKd}
    return RandomVector

def FindTheMostAtractiveFirefly(SwarmOfFireflies):
    IndexOfTheMostAtractiveFirefly = 0
    for i in range(len(SwarmOfFireflies)):
        if SwarmOfFireflies[i].getFitnessFunctionValue() < SwarmOfFireflies[IndexOfTheMostAtractiveFirefly].getFitnessFunctionValue():
            IndexOfTheMostAtractiveFirefly = i

    return IndexOfTheMostAtractiveFirefly

    # for firefly in SwarmOfFireflies:
    #     if firefly.getFitnessFunctionValue() < SwarmOfFireflies[IndexOfTheMostAtractiveFirefly].getFitnessFunctionValue():
    #         IndexOfTheMostAtractiveFirefly = firefly.get_index()
    #
    # return IndexOfTheMostAtractiveFirefly

def collectListOfPoints(SetOfObjects):
    listOfPoints=[]
    for i in range(len(SetOfObjects)):
        listOfPoints.append(SetOfObjects[i].get_Point())

    return listOfPoints

