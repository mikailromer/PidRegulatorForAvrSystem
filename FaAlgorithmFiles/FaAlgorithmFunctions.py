import numpy as np
from FaAlgorithmFiles.Firefly import Firefly
import sympy


def createSwarmOfFireflies(NumberOfFireflies, pidGainsThresholds, beta0):
    SwarmOfFireflies = []
    for index in range(NumberOfFireflies):
        Kp = round(np.random.uniform(0, pidGainsThresholds["Kp"]), 3)
        Ki = round(np.random.uniform(0, pidGainsThresholds["Ki"]), 3)
        Kd = round(np.random.uniform(0, pidGainsThresholds["Kd"]), 3)
        SwarmOfFireflies.append(Firefly(Kp, Ki, Kd, beta0))
    return SwarmOfFireflies

#def fitnessFunction(T, Vout, M, Ess, Ts, Tr, delta, ro ):



def ComputeDistanceBeetweenTwoObjects(Firefly_I, Firefly_J):
    if ((Firefly_I.get_X()==None and Firefly_I.get_Y()==None) or (Firefly_J.get_X()==None and Firefly_J.get_Y()==None)):
        return None
    else:
        X = Firefly_J.get_X() - Firefly_I.get_X()
        Y = Firefly_J.get_Y() - Firefly_I.get_Y()
        Rij = np.sqrt(X ** 2 + Y ** 2)
        return Rij

def AdaptationFunction(CostFunction,beta):
    if CostFunction==None:
        return None
    else:
        result = -beta *CostFunction
        return result

def AtractivenessFunction(beta0,Rij,Lambda):
    result=beta0*np.exp(-1*Lambda*(Rij**2))
    return result

def CostFunctionForPlot(X, Y):
     #return np.exp(np.sin(-np.sqrt(X**2+Y**2)))
     return X ** 2 + Y ** 2
     #return 1000*np.sin(X + np.log10(np.fabs(X) + 0.00001)) + X ** 2 + Y ** 2

def CostFunction(X, Y):
    if (X==None or Y==None):
        return None
    else:
        #return np.exp(np.sin(-np.sqrt(X ** 2 + Y ** 2)))
        #return 1000*np.sin(X+np.log10(np.fabs(X)+0.00001))+X**2+Y**2
        return X ** 2 + Y ** 2

def collectListOfPoints(SetOfObjects):
    listOfPoints=[]
    for i in range(len(SetOfObjects)):
        listOfPoints.append(SetOfObjects[i].get_Point())

    return listOfPoints