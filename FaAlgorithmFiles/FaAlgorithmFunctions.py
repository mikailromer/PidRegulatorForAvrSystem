import numpy as np
from FaAlgorithmFiles.Firefly import Firefly
from control import step_response
import sympy


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
        t = t + 0.01

    interpVout = np.interp(interpTime, T, Vout)
    return interpTime, interpVout

def ITAE(T, Vout):
    integral = 0
    for i in range(1,len(T)):
        deltaT = T[i] - T[i-1]
        field = ((Vout[i] + Vout[i-1]) * deltaT) /2
        integral = integral + field

    return integral


def fitnessFunction(T, Vout, M, Ess, Ts, Tr, ro):
    fitnessFunctionValue = ITAE(T, Vout) *((1 - np.exp(ro)) *(M + Ess) + np.exp(ro)*(Ts - Tr))
    return fitnessFunctionValue

def ligthIntensivity(fitnessFunctionValue):
    return np.exp(-fitnessFunctionValue)

def step_info(t,Vout):
    overshoot =(Vout.max()/Vout[-1]-1)
    # a =next(i for i in range(0,len(Vout)-1) if Vout[i]>Vout[-1]*.90)
    # for i in range(0, len(Vout) -1):
    #     if Vout[i] > Vout[-1] * .90:
    #         print(i)
    risingTime = t[next(i for i in range(0,len(Vout)-1) if Vout[i]>Vout[-1]*.90)]-t[0]
    for i in range(2, len(Vout) - 1):
        if abs(Vout[-i] / Vout[-1]) > 1.02:
            setTim = t[len(Vout)-i] - t[0]
            seting = t[i] - t[0]


    settingTime = t[next(len(Vout)-i for i in range(2,len(Vout)-1) if abs(Vout[-i]/Vout[-1])>1.02)]-t[0]
    return overshoot, risingTime, settingTime

def calculateFireflyLigthIntensivity(firefly, Ga, Ge, Gg, voltageReference, ro):
    firefly.setPidControllerTransferFunction(firefly.get_Kp(), firefly.get_Ki(), firefly.get_Kd())
    Gpid = firefly.getPidControllerTransferFunction()
    systemTransferFunction = (Gpid * Ga * Ge * Gg) / (1 + (Gpid * Ga * Ge * Gg))
    T, Vout = step_response(systemTransferFunction * voltageReference)
    interpTime, interpVout = interpolationOfOutputVoltage(T, Vout)
    Ess = np.abs(voltageReference - interpVout[len(interpVout) - 1])
    M, Tr, Ts = step_info(interpTime, interpVout)
    firefly.setAvrSystemResponseParameters(Ess, M, Tr, Ts, interpTime, interpVout)
    fitnessFunctionValue = fitnessFunction(interpTime,interpVout, M, Ess, Ts, Tr, ro)
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

def GenerateRandomVector():
    uKp = round(np.random.uniform(0, 1), 3) -0.5
    uKi = round(np.random.uniform(0, 1), 3) -0.5
    uKd = round(np.random.uniform(0, 1), 3) -0.5
    RandomVector = {"Kp": uKp, "Ki": uKi, "Kd":uKd}
    return RandomVector

def FindTheMostAtractiveFirefly(SwarmOfFireflies):
    IndexOfTheMostAtractiveFirefly = 0
    for firefly in SwarmOfFireflies:
        if firefly.get_Z() < SwarmOfFireflies[IndexOfTheMostAtractiveFirefly].get_Z():
            IndexOfTheMostAtractiveFirefly = firefly.get_index()

    return IndexOfTheMostAtractiveFirefly

def collectListOfPoints(SetOfObjects):
    listOfPoints=[]
    for i in range(len(SetOfObjects)):
        listOfPoints.append(SetOfObjects[i].get_Point())

    return listOfPoints

