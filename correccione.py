#!/usr/bin/env python3
################### Cuadro de importaciones ################################
import numpy as np
import matplotlib.pyplot as plt
import cv2
import argparse
from img_util import plt_cv_image, add_cols_left, add_cols_right, draw_points, hsv_to_cv
############################################################################
global nombreArchivo
nombreArchivo=''
global coordenadas
coordenadas=[]
################### Cuadro de Funciones ####################################

# Función para la lectura del video, recibe como párametro el nombre del video
def leerVideo(NombreArchivo):
    video = cv2.VideoCapture(NombreArchivo)                           # Funcionalidad de cv2 para importar el video 
    Frames = []                                                       # Lista en la cual, cada posicion corrsponde a la matriz de cada frame
    cto = 0
    while(video.isOpened()):                                          # Mientras el video no haya terminado, el ciclo continua
        ret, frame = video.read()                                     # obtiene el frame en el instante actual
        cto = cto +1
        if ret == True:
            Frames.append(frame)                                      # Lista para almacenar cada 
            #cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):                     # condición de salida 
                break
            if cto == 476:
                break
        else:
            break
    video.release()
    cv2.destroyAllWindows()
    #cv2.imshow('Primer Frame del video',Frames[0])
    #cv2.waitKey()
    return Frames                                                     # La función retorna la lista de frames del video

# Función para la determinación de las coordenadas de los pixeles correspondientes a las esquinas de la cancha
def Puntos1(NombreArchivo):
    global nombreArchivo
    nombreArchivo=NombreArchivo
    if NombreArchivo == 'ssl1.mp4':
        pts1 = np.float32([[285,30],[1315,95],[35,700],[1420,820]])   # Coordenadas esquinas video ssl1.mp4
    if NombreArchivo == 'ssl2.mp4':
        pts1 = np.float32([[220,53],[1255,25],[25,770],[1435,720]])   # Coordenadas esquinas video ssl2.mp4
    if NombreArchivo == 'ssl3.mp4':
        pts1 = np.float32([[124,120],[1115,15],[61,825],[1413,625]])  # Coordenadas esquinas video ssl3.mp4
    return pts1

# Función para la determinación de las coordenadas de los pixeles correspondientes a las esquinas de la cancha
def Puntos(NombreArchivo,Frames):
    global nombreArchivo
    global coordenadas
    nombreArchivo=NombreArchivo #Guarda el nombre del archivo por parametro
    cv2.namedWindow('Seleccionar puntos') #Crea una ventana
    cv2.setMouseCallback('Seleccionar puntos',obtenerPuntos) #Se esta a la espera de algun evento para la ventana definida
    cv2.imshow('Seleccionar puntos',Frames[0]) #Se muestra la ventana sobre la cual se escogeran los puntos
    cv2.waitKey()
    cv2.destroyAllWindows()
    return np.float32(coordenadas)

#Funcion que añade las coordenadas de cada click a un arreglo
def obtenerPuntos(event,x,y,flags,param):
    global coordenadas
    if event == 4: #Evento de un click izquierdo 
        coordenadas.append([x,y]) #Por cada click se añaden a una lista de coordenadas
        
# Función que permite realizar el cambio de perspectiva a una vista vertical
def CambioPerspectiva(Frames, pts1):
    img1 = Frames[0]                                                  # Primer frame del video
    draw_points(img1, pts1, '4 Puntos img1')                          # Permite graficar los puntos en el frame del video
    pts2 = np.float32([[0,0],[1461,0],[0,849],[1461,849]])            # Coordenadas para realizar el cambio de perspectiva
    M = cv2.getPerspectiveTransform(pts1,pts2)                        # Matriz de transformación de perspectiva 
    for i in range(len(Frames)):                                      # Cambio de perspectiva de cada Frame del video
        Frames[i] = cv2.warpPerspective(Frames[i], M, dsize = (Frames[i].shape[1],Frames[i].shape[0]))
    plt_cv_image(Frames[0], 'Imagen transformada')
    return Frames

# Función que permite realizar una mascara binaria a partir de limites en un esquema de color hsv
def BinarizarColores(hsv_limits, Frames):
    mask_list = []
    for i in range(len(Frames)):
        hsv = cv2.cvtColor(Frames[i], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_to_cv(hsv_limits[0]), hsv_to_cv(hsv_limits[1]))
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        mask_list.append(mask)
#     cv2.imshow('sdfs',Frames[412])
#     cv2.imshow('Mask',mask_list[412])
#     cv2.waitKey()
    return mask_list

# Funcion para determinar los contornos de la mascara binaria y posterirmente determinar el centro de cada circulo
def Contornos_y_centros(mask_list,Frames):
    centers_list = []
    for i in range(len(mask_list)):
        contours, hierarchy = cv2.findContours(mask_list[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centers = [cv2.minEnclosingCircle(c)[0] for c in contours] 
        for j in range(len(centers)):
            B = [int(element) for element in centers[j]]
            centers[j] = B
        centers_list.append(centers)
#         if i==412:
#             draw_points(Frames[412],centers_list[0], '4 Puntos img1')
    return centers_list

#Encuentra los puntos cercanos en un radio de 15px a un punto azul o amarillo para todos los frames, es decir agrupa los puntos que corresponden a 1 robot para todos los frames.
#Se retorna un arreglo en el cual cada fila corresponde a la información de un frame. Cada fila contiene un arreglo del tamaño del numero de puntos azules o amarillos (depende del parametro dado). Por cada punto azul o amarillo hay un arreglo de 3 posiciones, en la primera las coordenadas del circulo color azul o amarillo, en la segunda un arreglo de los puntos cercanos de color magenta y en la tercera un arreglo de puntos cercanos de color verde. 
def Encontrar_cercanos(coordenadas_centrales,coordenadas_Magenta,coordenadas_Verde):
    circulos_cercanos = []
    for i in range(len(coordenadas_centrales)): #Recorre los frames
        circulos_cercanos_actual = []
        for j in range(len(coordenadas_centrales[i])):#Recorre los puntos azules/amarillos de un frame
            magentas_adyacentes = []
            verdes_adyacentes = []
            for k in range(len(coordenadas_Magenta[i])):#Recorre todos los puntos magenta para un frame y verifica si la distancia al punto                                                          azul/amarillo es menor a 15px. Si lo es lo guarda junto con el punto central
                distancia_magenta = ((coordenadas_centrales[i][j][0]- coordenadas_Magenta[i][k][0])**2 +(coordenadas_centrales[i][j][1]- coordenadas_Magenta[i][k][1])**2)**(1/2)
                if distancia_magenta <= 18.5:
                    magentas_adyacentes.append(coordenadas_Magenta[i][k])
                        
            for k in range(len(coordenadas_Verde[i])):#Recorre todos los puntos verdes para un frame y verifica si la distancia al punto                                                          azul/amarillo es menor a 15px. Si lo es lo guarda junto con el punto central
                distancia_verde = ((coordenadas_centrales[i][j][0]- coordenadas_Verde[i][k][0])**2 +(coordenadas_centrales[i][j][1]- coordenadas_Verde[i][k][1])**2)**(1/2)
                if distancia_verde <= 18.5:
                        verdes_adyacentes.append(coordenadas_Verde[i][k])          
            circulos_cercanos_actual.append([coordenadas_centrales[i][j],magentas_adyacentes,verdes_adyacentes] )       
        circulos_cercanos.append(circulos_cercanos_actual)
    return circulos_cercanos

#Encuentra el ID de todos los robots de un color segun el parametro para todos los frames
#Para distingir entre el id 0 y 2 se cuentan cuantos colores verdes o magentas hay en cada robot, si hay mas de 2 verdes es id 2, si hay mas de 2 magentas es id 0.
#Para distinguir entre id 1 y 3 era necesario distinguir la distancia entre puntos, por lo que bajo cierto umbral de distancia entre puntos verdes, si estan mas cercanos puede clasificarse como id 1 y si estan mas lejanos se puede clasificar como id 3.
#Retorna un arreglo en el cual cada fila hace referencia a un frame, en cada fila hay un arreglo de puntos en el cual la primera posición hace referencia al id del robot que corresponde a la coordenada central de la segunda posicion.
def definirId(cercanos,Frames):
    idFrame = []
    for i in range(len(cercanos)):
        idRobot=[]        
        for j in range(len(cercanos[i])):
            coordenadas_central = cercanos[i][j][0] #Coordenada central del robot que define el color del equipo
            Lista_coordenadas_magenta = cercanos[i][j][1]#Arreglo de coordenadas magenta que pertenecen al mismo robot
            Lista_coordenadas_verde = cercanos[i][j][2]#Arreglo de coordenadas verdes que pertenecen al mismo robot
            numMagentas=len(Lista_coordenadas_magenta)
            numVerdes=len(Lista_coordenadas_verde)
            if numMagentas>2:
                idRobot.append([0,coordenadas_central])
            elif numVerdes>2:
                idRobot.append([2,coordenadas_central])
            elif numVerdes+ numMagentas!=4:
                print('verdes ',numVerdes)
                print('magenta ',numMagentas)
                print(i)   
            else:
                #print('verde',Lista_coordenadas_verde)
                #print('magenta',Lista_coordenadas_magenta)
                distancia = ((Lista_coordenadas_verde[0][0]- Lista_coordenadas_verde[1][0])**2 +(Lista_coordenadas_verde[0][1]- Lista_coordenadas_verde[1][1])**2)**(1/2)
                if distancia<25:
                    idRobot.append([1,coordenadas_central])
                else:
                    idRobot.append([3,coordenadas_central])
        idFrame.append(idRobot)
    return idFrame  

#Ahora se escriben los ids de cada robot sobre cada coordenada central del robot para todos los frames
def pintarIds(idsAzul, idsAmarillo,Frames):
    font = cv2.FONT_HERSHEY_SIMPLEX 
    fontScale = 1
    color = (57, 255, 20) 
    thickness = 2
    for i in range(len(Frames)):
        for j in range(len(idsAzul[i])):
            cv2.putText(Frames[i], str(idsAzul[i][j][0]), (idsAzul[i][j][1][0]+20,idsAzul[i][j][1][1]+20), font,fontScale,color,thickness,cv2.LINE_AA) 
        for k in range(len(idsAmarillo[i])):
            cv2.putText(Frames[i], str(idsAmarillo[i][k][0]), (idsAmarillo[i][k][1][0]+20,idsAmarillo[i][k][1][1]+20), font,fontScale,color,thickness,cv2.LINE_AA) 
#         if i==0:
#             print(len(Frames[i]))
#             print(len(Frames[i][0]))
#             cv2.imshow('orden',Frames[i])
#             cv2.waitKey()
    return Frames

def resizeFrames(Frames):
    for i in range(len(Frames)):
        Frames[i]=cv2.resize(Frames[i],(900,600))
    return Frames

def guardarVideo(Frames):
    global nombreArchivo
    size=(len(Frames[0][0]),len(Frames[0]))
    print(size)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('nuevo_'+nombreArchivo,fourcc, 30, size)
    
    for i in range(len(Frames)):
        out.write(Frames[i])
    out.release()
                    
############################################################################   

###################### Método Principal  ###################################

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("NombreArchivo", type= str, help="Nombre archivo mp4") # El nombre del archivo mp4 ingrsado por párametro
    args = parser.parse_args()                                                 # objeto de entrada que se pasa cmo argumento
    Frames = leerVideo(args.NombreArchivo)                                     # Lista de Frames del video
    pts=Puntos(args.NombreArchivo,Frames)
    Frames = CambioPerspectiva(Frames, pts)                                    # Lista de Frames del video con el cambio de perspectiva
    print('Cambio de perspectiva')
    Azul = [ [210,30,30], [270,100,100] ]                                      # Límites de color en el espacio hsv para el azul 
    #Amarillo = [ [20,40,40], [110,100,100] ]                                   # Límites de color en el espacio hsv para el amarillo
    Amarillo = [ [46,80,100], [68,100,100] ] 
    #Magenta = [ [250,20,20], [360,100,100] ]                                   # Límites de color en el espacio hsv para el magenta
    #Verde = [ [110,20,50], [150,100,100] ]                                     # Límites de color en el espacio hsv para el verde
    Verde = [ [90,20,50], [160,100,100] ]  
    Magenta = [ [280,20,20], [360,100,100] ]
    mask_Azul = BinarizarColores(Azul,Frames)                                  # lista de mascara del color azul para cada frame
    mask_Amarillo = BinarizarColores(Amarillo,Frames)                          # lista de mascara del color amarillo para cada frame 
    mask_Magenta = BinarizarColores(Magenta,Frames)                            # lista de mascara del color Magenta para cada frame  
    mask_Verde = BinarizarColores(Verde,Frames)                                # lista de mascara del color verde para cada frame
    coordenadas_Azul = Contornos_y_centros(mask_Azul,Frames)                          # lista de coordenadas del color azul para cada frame 
    coordenadas_Amarillo = Contornos_y_centros(mask_Amarillo,Frames)                  # lista de coordenadas del color amarillo para cada frame 
    coordenadas_Magenta = Contornos_y_centros(mask_Magenta,Frames)                    # lista de coordenadas del color magenta para cada frame     
    coordenadas_Verde = Contornos_y_centros(mask_Verde,Frames)                        # lista de coordenadas del color verde para cada frame
    
    
    # Cercanos azul/amarillo es una lista de frames donde en cada frame hay una lista que en la primera posición tiene las coordenandas del     punto azul, en la segunda posición una lista de coordenadas de puntos magenta (puntos menores a 15 pixeles) y en la tercera posición       una lista de coordenadas de puntos verdes (puntos menores a 15 pixeles), para cada punto azul.
    cercanos_azul = Encontrar_cercanos(coordenadas_Azul,coordenadas_Magenta,coordenadas_Verde)
    cercanos_amarillo = Encontrar_cercanos(coordenadas_Amarillo,coordenadas_Magenta,coordenadas_Verde)
    idsAzul=definirId(cercanos_azul,Frames)                                           # lista de ids de los robots del equipo azul para cada frame
    idsAmarillo=definirId(cercanos_amarillo,Frames)                                   # lista de ids de los robots del equipo azul para cada frame
    Frames=pintarIds(idsAzul,idsAmarillo,Frames)                               # Muestra los ids de cada robot por cada frame
    Frames=resizeFrames(Frames) 
    guardarVideo(Frames)
############################################################################ 
