from cv2 import dft
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics as metric
from math import sqrt
import statistics

dfPri = pd.read_csv (r"pri.csv", sep=";")
dfSeq = pd.read_csv (r"seg.csv", sep=";")

# print(dfPercurso.tail())

# -------------------------------- QUADRADO -------------------------------------------------------------------------
def Plot3DGPSChildPontosQuadrado():
    xlineGPSQuadrado = dfQuadrado['GPSChildX'].values
    ylineGPSQuadrado = dfQuadrado['GPSChildY'].values
    zlineGPSQuadrado = dfQuadrado['GPSChildZ'].values

    xlineTagQuadrado = dfQuadrado['GPSMotherX'].values + dfQuadrado['tagX'].values
    ylineTagQuadrado = dfQuadrado['GPSMotherY'].values + dfQuadrado['tagY'].values 
    zlineTagQuadrado = dfQuadrado['GPSMotherZ'].values - dfQuadrado['tagZ'].values

    xlinePontos = dfQuadrado['pontosX'].values
    ylinePontos = dfQuadrado['pontosY'].values
    zlinePontos = dfQuadrado['pontosZ'].values - 3

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.plot3D(xlineGPSQuadrado, ylineGPSQuadrado, zlineGPSQuadrado, 'blue', label='UAV Position')
    ax.plot3D(xlinePontos, ylinePontos, zlinePontos, 'green', label='Planned route')
    ax.plot3D(xlineTagQuadrado, ylineTagQuadrado, zlineTagQuadrado, 'red', label='Tag readings')

    ax.set_title("3D Route")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.set_zlabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_zlim3d([2, 4])

def PlotXYGPSChildPontosQuadrado():
    xlineGPSQuadrado = dfQuadrado['GPSChildX'].values
    ylineGPSQuadrado = dfQuadrado['GPSChildY'].values

    xlineTagQuadrado = dfQuadrado['GPSMotherX'].values + dfQuadrado['tagX'].values
    ylineTagQuadrado = dfQuadrado['GPSMotherY'].values + dfQuadrado['tagY'].values
    
    xlinePontos = dfQuadrado['pontosX'].values
    ylinePontos = dfQuadrado['pontosY'].values

    fig = plt.figure()
    ax = plt.axes()

    ax.plot(xlineGPSQuadrado, ylineGPSQuadrado, 'blue', label='UAV Position')
    ax.plot(xlinePontos, ylinePontos, 'green', label='Planned route')
    ax.plot(xlineTagQuadrado, ylineTagQuadrado, 'red', label='Tag readings')
    ax.set_title("Route in X and Y position")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_xlim([-1.5, 2])
    ax.set_ylim([-1.5, 2])

def PlotZGPSChildPontosQuadrado():
    zlineGPSQuadrado = dfQuadrado['GPSChildZ'].values
    indexLineGPSQuadrado = dfQuadrado['GPSChildZ'].index

    zlineTagQuadrado = dfQuadrado['GPSMotherZ'].values - dfQuadrado['tagZ'].values
    indexLineTagQuadrado = dfQuadrado['tagZ'].index
    
    zlinePontos = dfQuadrado['pontosZ'].values - 3
    indexLinePontos = dfQuadrado['pontosZ'].index

    fig = plt.figure()
    ax = plt.axes()

    ax.plot(indexLineGPSQuadrado, zlineGPSQuadrado, 'blue', label='UAV Position')
    ax.plot(indexLinePontos, zlinePontos, 'green', label='Planned route')
    ax.plot(indexLineTagQuadrado, zlineTagQuadrado, 'red', label='Tag readings')
    ax.set_title("Route in Z position")
    ax.set_ylabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')

def ErrosQuadrado():
    xTrue = dfQuadrado['GPSChildX'].values
    yTrue = dfQuadrado['GPSChildY'].values
    zTrue = dfQuadrado['GPSChildZ'].values

    xTest = dfQuadrado['GPSMotherX'].values + dfQuadrado['tagX'].values
    yTest = dfQuadrado['GPSMotherY'].values + dfQuadrado['tagY'].values 
    zTest = dfQuadrado['GPSMotherZ'].values - dfQuadrado['tagZ'].values

    mseX = metric.mean_squared_error(xTrue, xTest)
    rmseX = sqrt(mseX)

    mseY = metric.mean_squared_error(yTrue, yTest)
    rmseY = sqrt(mseY)

    mseZ = metric.mean_squared_error(zTrue, zTest)
    rmseZ = sqrt(mseZ)

    erroMaxX = metric.max_error(xTrue, xTest)
    erroMaxY = metric.max_error(yTrue, yTest)
    erroMaxZ = metric.max_error(zTrue, zTest)

    mediaX = statistics.mean(xTest)
    mediaY = statistics.mean(yTest)
    mediaZ = statistics.mean(zTest)

    stDevX = statistics.stdev(xTest)
    stDevY = statistics.stdev(yTest)
    stDevZ = statistics.stdev(zTest)

    medianaX = statistics.median(xTest)
    medianaY = statistics.median(yTest)
    smedianaZ = statistics.median(zTest)

    maxX = xTest.max()
    maxY = yTest.max()
    maxZ = zTest.max()

    minX = xTest.min()
    minY = yTest.min()
    minZ = zTest.min()

    print("\n---------------------------------------------------------------------------------------")
    print("QUADRADO")
    print("ERRO QUADRATICO MEDIO -> X=", mseX, "   Y=", mseY, "   Z=", mseZ )
    print("RAIZ QUADRADA DO ERRO MEDIO -> X=", rmseX, "   Y=", rmseY, "   Z=", rmseZ )
    print("ERRO MAXIMO -> X=", erroMaxX, "   Y=", erroMaxY, "   Z=", erroMaxZ )
    print("---------------------------------------------------------------------------------------\n")
# ---------------------------------------------------------------------------------------------------------

# -------------------------------- XYZ -------------------------------------------------------------------------

def PlotXYZ():
    xlinePri = dfPri['pontosX'].values
    ylinePri = dfPri['pontosY'].values
    zlinePri = dfPri['pontosZ'].values

    xlineSeq = dfSeq['pontosX'].values
    ylineSeq = dfSeq['pontosY'].values
    zlineSeq = dfSeq['pontosZ'].values

    xline = [0.0, 0.45, -0.45, -0.45, 0.45, 0.45, 0.0]
    yline = [0.0, 0.45, 0.45, -0.15, -0.15, 0.45, 0.0]
    zline = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, ]

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    #ax.plot3D(xlinePri, ylinePri, zlinePri, 'blue', label='UAV Position')
    ax.plot3D(xlineSeq, ylineSeq, zlineSeq, 'red', label='UAV Position')
    ax.plot3D(xline, yline, zline, 'green', label='Planned route')

    ax.set_title("3D Route")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.set_zlabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_zlim3d([-3, 3])

# ---------------------------------------------------------------------------------------------------------

# -------------------------------- ZIGZAG -------------------------------------------------------------------------
def Plot3DGPSChildPontosZigzag():
    xlineGPS = dfZigzag['GPSChildX'].values
    ylineGPS = dfZigzag['GPSChildY'].values
    zlineGPS = dfZigzag['GPSChildZ'].values

    xlineTag = dfZigzag['GPSMotherX'].values + dfZigzag['tagX'].values
    ylineTag = dfZigzag['GPSMotherY'].values + dfZigzag['tagY'].values 
    zlineTag = dfZigzag['GPSMotherZ'].values - dfZigzag['tagZ'].values

    xlinePontos = dfZigzag['pontosX'].values
    ylinePontos = dfZigzag['pontosY'].values
    zlinePontos = dfZigzag['pontosZ'].values - 3

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.plot3D(xlineGPS, ylineGPS, zlineGPS, 'blue', label='UAV Position')
    ax.plot3D(xlinePontos, ylinePontos, zlinePontos, 'green', label='Planned route')
    ax.plot3D(xlineTag, ylineTag, zlineTag, 'red', label='Tag readings')

    ax.set_title("3D Route")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.set_zlabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_zlim3d([2, 4])

def PlotXYGPSChildPontosZigzag():
    xlineGPS = dfZigzag['GPSChildX'].values
    ylineGPS = dfZigzag['GPSChildY'].values

    xlineTag = dfZigzag['GPSMotherX'].values + dfZigzag['tagX'].values
    ylineTag = dfZigzag['GPSMotherY'].values + dfZigzag['tagY'].values
    
    xlinePontos = dfZigzag['pontosX'].values
    ylinePontos = dfZigzag['pontosY'].values

    fig = plt.figure()
    ax = plt.axes()

    ax.plot(xlineGPS, ylineGPS, 'blue', label='UAV Position')
    ax.plot(xlinePontos, ylinePontos, 'green', label='Planned route')
    ax.plot(xlineTag, ylineTag, 'red', label='Tag readings')
    ax.set_title("Route in X and Y position")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_xlim([-1.5, 2])
    ax.set_ylim([-1.5, 2])

def PlotZGPSChildPontosZigzag():
    zlineGPS = dfZigzag['GPSChildZ'].values
    indexLineGPS = dfZigzag['GPSChildZ'].index

    zlineTag = dfZigzag['GPSMotherZ'].values - dfZigzag['tagZ'].values
    indexLineTag = dfZigzag['tagZ'].index
    
    zlinePontos = dfZigzag['pontosZ'].values - 3
    indexLinePontos = dfZigzag['pontosZ'].index

    fig = plt.figure()
    ax = plt.axes()

    ax.plot(indexLineGPS, zlineGPS, 'blue', label='UAV Position')
    ax.plot(indexLinePontos, zlinePontos, 'green', label='Planned route')
    ax.plot(indexLineTag, zlineTag, 'red', label='Tag readings')
    ax.set_title("Route in Z position")
    ax.set_ylabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')

def ErrosZigzag():
    xTrue = dfZigzag['GPSChildX'].values
    yTrue = dfZigzag['GPSChildY'].values
    zTrue = dfZigzag['GPSChildZ'].values

    xTest = dfZigzag['GPSMotherX'].values + dfZigzag['tagX'].values
    yTest = dfZigzag['GPSMotherY'].values + dfZigzag['tagY'].values 
    zTest = dfZigzag['GPSMotherZ'].values - dfZigzag['tagZ'].values

    mseX = metric.mean_squared_error(xTrue, xTest)
    rmseX = sqrt(mseX)

    mseY = metric.mean_squared_error(yTrue, yTest)
    rmseY = sqrt(mseY)

    mseZ = metric.mean_squared_error(zTrue, zTest)
    rmseZ = sqrt(mseZ)

    erroMaxX = metric.max_error(xTrue, xTest)
    erroMaxY = metric.max_error(yTrue, yTest)
    erroMaxZ = metric.max_error(zTrue, zTest)

    mediaX = statistics.mean(xTest)
    mediaY = statistics.mean(yTest)
    mediaZ = statistics.mean(zTest)

    stDevX = statistics.stdev(xTest)
    stDevY = statistics.stdev(yTest)
    stDevZ = statistics.stdev(zTest)

    medianaX = statistics.median(xTest)
    medianaY = statistics.median(yTest)
    smedianaZ = statistics.median(zTest)

    maxX = xTest.max()
    maxY = yTest.max()
    maxZ = zTest.max()

    minX = xTest.min()
    minY = yTest.min()
    minZ = zTest.min()

    print("\n---------------------------------------------------------------------------------------")
    print("ZIG ZAG")
    print("ERRO QUADRATICO MEDIO -> X=", mseX, "   Y=", mseY, "   Z=", mseZ )
    print("RAIZ QUADRADA DO ERRO MEDIO -> X=", rmseX, "   Y=", rmseY, "   Z=", rmseZ )
    print("ERRO MAXIMO -> X=", erroMaxX, "   Y=", erroMaxY, "   Z=", erroMaxZ )
    print("---------------------------------------------------------------------------------------\n")
# ---------------------------------------------------------------------------------------------------------

# -------------------------------- PERCURSO -------------------------------------------------------------------------
def Plot3DGPSChildPontosPercurso():
    xlineGPS = dfPercurso['GPSChildX'].values
    ylineGPS = dfPercurso['GPSChildY'].values
    zlineGPS = dfPercurso['GPSChildZ'].values

    xlineTag = dfPercurso['GPSMotherX'].values + dfPercurso['tagX'].values
    ylineTag = dfPercurso['GPSMotherY'].values + dfPercurso['tagY'].values 
    zlineTag = dfPercurso['GPSMotherZ'].values - dfPercurso['tagZ'].values

    xlinePontos = dfPercurso['pontosX'].values
    ylinePontos = dfPercurso['pontosY'].values
    zlinePontos = dfPercurso['pontosZ'].values - 3

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.plot3D(xlineGPS, ylineGPS, zlineGPS, 'blue', label='UAV Position')
    ax.plot3D(xlinePontos, ylinePontos, zlinePontos, 'green', label='Planned route')
    ax.plot3D(xlineTag, ylineTag, zlineTag, 'red', label='Tag readings')

    ax.set_title("3D Route")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.set_zlabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_zlim3d([2, 4])

def PlotXYGPSChildPontosPercurso():
    xlineGPS = dfPercurso['GPSChildX'].values
    ylineGPS = dfPercurso['GPSChildY'].values

    xlineTag = dfPercurso['GPSMotherX'].values + dfPercurso['tagX'].values
    ylineTag = dfPercurso['GPSMotherY'].values + dfPercurso['tagY'].values
    
    xlinePontos = dfPercurso['pontosX'].values
    ylinePontos = dfPercurso['pontosY'].values

    fig = plt.figure()
    ax = plt.axes()

    ax.plot(xlineGPS, ylineGPS, 'blue', label='UAV Position')
    ax.plot(xlinePontos, ylinePontos, 'green', label='Planned route')
    ax.plot(xlineTag, ylineTag, 'red', label='Tag readings')
    ax.set_title("Route in X and Y position")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_xlim([-1.5, 2])
    ax.set_ylim([-1.5, 2])

def PlotZGPSChildPontosPercurso():
    zlineGPS = dfPercurso['GPSChildZ'].values
    indexLineGPS = dfPercurso['GPSChildZ'].index

    zlineTag = dfPercurso['GPSMotherZ'].values - dfPercurso['tagZ'].values
    indexLineTag = dfPercurso['tagZ'].index
    
    zlinePontos = dfPercurso['pontosZ'].values - 3
    indexLinePontos = dfPercurso['pontosZ'].index

    fig = plt.figure()
    ax = plt.axes()

    ax.plot(indexLineGPS, zlineGPS, 'blue', label='UAV Position')
    ax.plot(indexLinePontos, zlinePontos, 'green', label='Planned route')
    ax.plot(indexLineTag, zlineTag, 'red', label='Tag readings')
    ax.set_title("Route in Z position")
    ax.set_ylabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')

def ErrosPercurso():
    xTrue = dfPercurso['GPSChildX'].values
    yTrue = dfPercurso['GPSChildY'].values
    zTrue = dfPercurso['GPSChildZ'].values

    xTest = dfPercurso['GPSMotherX'].values + dfPercurso['tagX'].values
    yTest = dfPercurso['GPSMotherY'].values + dfPercurso['tagY'].values 
    zTest = dfPercurso['GPSMotherZ'].values - dfPercurso['tagZ'].values

    mseX = metric.mean_squared_error(xTrue, xTest)
    rmseX = sqrt(mseX)

    mseY = metric.mean_squared_error(yTrue, yTest)
    rmseY = sqrt(mseY)

    mseZ = metric.mean_squared_error(zTrue, zTest)
    rmseZ = sqrt(mseZ)

    erroMaxX = metric.max_error(xTrue, xTest)
    erroMaxY = metric.max_error(yTrue, yTest)
    erroMaxZ = metric.max_error(zTrue, zTest)

    mediaX = statistics.mean(xTest)
    mediaY = statistics.mean(yTest)
    mediaZ = statistics.mean(zTest)

    stDevX = statistics.stdev(xTest)
    stDevY = statistics.stdev(yTest)
    stDevZ = statistics.stdev(zTest)

    medianaX = statistics.median(xTest)
    medianaY = statistics.median(yTest)
    smedianaZ = statistics.median(zTest)

    maxX = xTest.max()
    maxY = yTest.max()
    maxZ = zTest.max()

    minX = xTest.min()
    minY = yTest.min()
    minZ = zTest.min()

    print("\n---------------------------------------------------------------------------------------")
    print("PERCURSO")
    print("ERRO QUADRATICO MEDIO -> X=", mseX, "   Y=", mseY, "   Z=", mseZ )
    print("RAIZ QUADRADA DO ERRO MEDIO -> X=", rmseX, "   Y=", rmseY, "   Z=", rmseZ )
    print("ERRO MAXIMO -> X=", erroMaxX, "   Y=", erroMaxY, "   Z=", erroMaxZ )
    print("---------------------------------------------------------------------------------------\n")
# ---------------------------------------------------------------------------------------------------------

# -------------------------------- FOLLOW -------------------------------------------------------------------------
# dfFollowOne = pd.read_csv (r"src\tccDrones\logsPos\logFollow1.csv", sep=";")
# dfFollowTwo = pd.read_csv (r"src\tccDrones\logsPos\logFollow2.csv", sep=";")

def Plot3DFollowOne():
    xlineGPS = dfFollowOne['GPSChildX'].values
    ylineGPS = dfFollowOne['GPSChildY'].values
    zlineGPS = dfFollowOne['GPSChildZ'].values

    xlineTag = dfFollowOne['GPSMotherX'].values + dfFollowOne['tagX'].values
    ylineTag = dfFollowOne['GPSMotherY'].values + dfFollowOne['tagY'].values 
    zlineTag = dfFollowOne['GPSMotherZ'].values - dfFollowOne['tagZ'].values

    xlinePontos = dfFollowOne['GPSMotherX'].values
    ylinePontos = dfFollowOne['GPSMotherY'].values
    zlinePontos = dfFollowOne['GPSMotherZ'].values 

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.plot3D(xlineGPS, ylineGPS, zlineGPS, 'blue', label='Slave UAV Position')
    ax.plot3D(xlinePontos, ylinePontos, zlinePontos, 'green', label='Master UAV Position')
    ax.plot3D(xlineTag, ylineTag, zlineTag, 'red', label='Tag readings')

    ax.set_title("3D Route")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.set_zlabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_zlim3d([2, 10])

def Plot3DFollowTwo():
    xlineGPS = dfFollowTwo['GPSChildX'].values
    ylineGPS = dfFollowTwo['GPSChildY'].values
    zlineGPS = dfFollowTwo['GPSChildZ'].values

    xlineTag = dfFollowTwo['GPSMotherX'].values + dfFollowTwo['tagX'].values
    ylineTag = dfFollowTwo['GPSMotherY'].values + dfFollowTwo['tagY'].values 
    zlineTag = dfFollowTwo['GPSMotherZ'].values - dfFollowTwo['tagZ'].values

    xlinePontos = dfFollowTwo['GPSMotherX'].values
    ylinePontos = dfFollowTwo['GPSMotherY'].values
    zlinePontos = dfFollowTwo['GPSMotherZ'].values 

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.plot3D(xlineGPS, ylineGPS, zlineGPS, 'blue', label='Slave UAV Position')
    ax.plot3D(xlinePontos, ylinePontos, zlinePontos, 'green', label='Master UAV Position')
    ax.plot3D(xlineTag, ylineTag, zlineTag, 'red', label='Tag readings')

    ax.set_title("3D Route")
    ax.set_xlabel('Pose in $X$', fontsize=12)
    ax.set_ylabel('Pose in $Y$', fontsize=12)
    ax.set_zlabel('Pose in $Z$', fontsize=12)
    ax.legend(loc='best', shadow=True, fontsize='small')
    ax.set_zlim3d([2, 10])
# ---------------------------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------------------------------
def main():
    print(dfPri)
    print(dfSeq)
    # ErrosQuadrado()
    # Plot3DGPSChildPontosQuadrado()
    # PlotXYGPSChildPontosQuadrado()
    # PlotZGPSChildPontosQuadrado()
    
    # ErrosZigzag()
    # Plot3DGPSChildPontosZigzag()
    # PlotXYGPSChildPontosZigzag()
    # PlotZGPSChildPontosZigzag()

    # ErrosPercurso()
    # Plot3DGPSChildPontosPercurso()
    # PlotXYGPSChildPontosPercurso()
    # PlotZGPSChildPontosPercurso()

    # Plot3DFollowOne()
    # Plot3DFollowTwo()    
    
    PlotXYZ()

    plt.show()

main()
# ---------------------------------------------------------------------------------------------------------
