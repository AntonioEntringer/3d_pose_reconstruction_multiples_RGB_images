'''
Neste trabalho vocês deverão detectar o robô nos vídeos das 4 câmeras do espaço inteligente e obter a reconstrução da sua posição 3D no mundo. Feito isso, vocês deverão gerar um gráfico da posição do robô, 
mostrando a trajetória que ele realizou.

Para detectar o robô será usado um marcador ARUCO acoplado a sua plataforma. Rotinas de detecção desse tipo de marcador poderão ser usadas para obter sua posição central, assim como as suas quinas nas imagens.
Essas informações, juntamente com os dados de calibração das câmeras, poderão ser usadas para localização 3D do robô.

Informações a serem consideradas:

- Só é necessário a reconstrução do ponto central do robô (ou suas quinas, se vocês acharem melhor). Para isso, vocês podem usar o método explicado no artigo fornecido como material 
adicional ou nos slides que discutimos em sala de aula.

- O robô está identificado por um marcador do tipo ARUCO - Código ID 0 (zero) - Tamanho 30 x 30 cm

- Os vídeos estão sincronizados para garantir que, a cada quadro, vocês estarão processando imagens do robô capturadas no mesmo instante.

- A calibração das câmeras é fornecida em 4 arquivos no formato JSON (Junto com os arquivos JSON estou fornecendo uma rotina para leitura e extração dos dados de calibração).

- Rotinas de detecção dos marcadores Aruco em imagens e vídeo são fornecidas para ajudar no desenvolvimento do trabalho.

ATENÇÃO: Existem rotinas de detecção de ARUCO que já fornecem sua localização e orientação 3D, se a calibração da câmera e o tamanho do padrão forem fornecidas. 
Essas rotinas poderão ser usadas para fazer comparações com a reconstrução 3D fornecida pelo trabalho de vocês, mas não serão aceitas como o trabalho a ser feito. 
Portanto, lembrem-se que vocês deverão desenvolver a rotina de reconstrução, a partir da detecção do ARUCO acoplado ao robô nas imagens 2D capturadas nos vídeos.


DATA DE ENTREGA: 17/03/2023
'''

import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
import json

def reconstruct_3d_position(corners_list, K_list, R_list, T_list):
    """
    Reconstrói a posição 3D do marcador Aruco com base nas projeções em múltiplas câmeras.
    """
    A = []

    for i in range(len(corners_list)):
        if corners_list[i] is None:
            continue  # Ignora câmeras que não detectaram o Aruco
        
        # Coordenadas do centro do Aruco na imagem (pixels)
        u, v = corners_list[i]
        
        # Use a matriz de projeção correta, assumindo que T é a posição da câmera no mundo
        P = K_list[i] @ np.hstack((R_list[i].T, -R_list[i].T @ T_list[i]))
        
        # Cria as equações lineares para u e v
        A.append(u * P[2, :] - P[0, :])
        A.append(v * P[2, :] - P[1, :])
    
    if len(A) < 4:
        return None  # Precisa de pelo menos 2 câmeras para triangulação (2 eqs/câmera)
    
    A = np.vstack(A)
    
    # Resolver pelo SVD (última linha de Vt é a solução)
    _, _, Vt = np.linalg.svd(A)
    X_homog = Vt[-1]
    
    # Converter coordenadas homogêneas para cartesianas
    X = X_homog[:3] / X_homog[3]
    return X.flatten()


def plot_camera_axes(R, T, ax, scale=0.5, label=''):
    """
    Plota os eixos do referencial da câmera.
    - R: matriz de rotação (3x3), onde cada coluna representa um eixo (x, y, z)
    - T: vetor de translação (3x1), posição da câmera no ambiente
    - ax: objeto Axes3D onde será plotado
    - scale: comprimento das setas para visualização
    - label: rótulo para identificar a câmera
    """
    # Converte T para array 1D
    origin = T.flatten()
    
    # Vetores dos eixos da câmera em coordenadas globais
    x_axis = R[:, 0]
    y_axis = R[:, 1]
    z_axis = R[:, 2]
    
    # Plota os eixos usando ax.quiver
    ax.quiver(origin[0], origin[1], origin[2],
              x_axis[0], x_axis[1], x_axis[2],
              color='r', length=scale, normalize=True)
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis[0], y_axis[1], y_axis[2],
              color='g', length=scale, normalize=True)
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis[0], z_axis[1], z_axis[2],
              color='b', length=scale, normalize=True)
    
    # Adiciona um rótulo na posição da câmera
    ax.text(origin[0], origin[1], origin[2], label, fontsize=10)


# Function to read the intrinsic and extrinsic parameters of each camera
def camera_parameters(file):
    camera_data = json.load(open(file))
    K = np.array(camera_data['intrinsic']['doubles']).reshape(3, 3)
    res = [camera_data['resolution']['width'],
           camera_data['resolution']['height']]
    tf = np.array(camera_data['extrinsic']['tf']['doubles']).reshape(4, 4)
    R = tf[:3, :3]
    T = tf[:3, 3].reshape(3, 1)
    dis = np.array(camera_data['distortion']['doubles'])
    return K, R, T, res, dis

#Load cameras parameters
K0, R0, T0, res0, dis0 = camera_parameters('0.json')
K1, R1, T1, res1, dis1 = camera_parameters('1.json')
K2, R2, T2, res2, dis2 = camera_parameters('2.json')
K3, R3, T3, res3, dis3 = camera_parameters('3.json')

print('Camera 0\n')
print('Resolucao',res0,'\n')
print('Parametros intrinsecos:\n', K0, '\n')
print('Parametros extrinsecos:\n')
print('R0\n', R0, '\n')
print('T0\n', T0, '\n')
print('Distorcao Radial:\n', dis0)

parameters = aruco.DetectorParameters()
dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoDetector = aruco.ArucoDetector(dictionary, parameters)

vid1 = cv2.VideoCapture('camera-00.mp4')
vid2 = cv2.VideoCapture('camera-01.mp4')
vid3 = cv2.VideoCapture('camera-02.mp4')
vid4 = cv2.VideoCapture('camera-03.mp4')

#####################################PLOT###############################################
# Cria figura e eixo 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plota o ponto (0,0,0) em preto (origem do ambiente)
ax.scatter(0, 0, 0, color='k', s=50,marker='x')

#plot das cameras no espaço
plot_camera_axes(R0, T0, ax, scale=0.5, label='Câmera 0')
plot_camera_axes(R1, T1, ax, scale=0.5, label='Câmera 1')
plot_camera_axes(R2, T2, ax, scale=0.5, label='Câmera 2')
plot_camera_axes(R3, T3, ax, scale=0.5, label='Câmera 3')

# Configurações adicionais do gráfico
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Orientação das Câmeras no Ambiente')

trajetoria, = ax.plot([], [], [], marker='o', linestyle='--', color='r', label='Trajetória') #plot da trajetória

plt.ion() #Plot iterativo (estudar o que é isso)
plt.pause(0.05)
X_hist = np.empty((0, 3))
#####################################################################################

while True:
    _, img1 = vid1.read()
    _, img2 = vid2.read()
    _, img3 = vid3.read()
    _, img4 = vid4.read()
    
    if img1 is None and img2 is None and img3 is None and img4 is None:
        print("Empty Frame")
        break

    # Detecta os marcadores no frame
    corners1, ids1, _ = arucoDetector.detectMarkers(img1)   
    corners2, ids2, _ = arucoDetector.detectMarkers(img2)
    corners3, ids3, _ = arucoDetector.detectMarkers(img3)
    corners4, ids4, _ = arucoDetector.detectMarkers(img4)

    
    frame_markers1 = aruco.drawDetectedMarkers(img1.copy(), corners1, ids1) # Desenha todos os marcadores detectados na imagem
    frame_markers2 = aruco.drawDetectedMarkers(img2.copy(), corners2, ids2) # Sem utilidade para o funcionamento
    frame_markers3 = aruco.drawDetectedMarkers(img3.copy(), corners3, ids3)
    frame_markers4 = aruco.drawDetectedMarkers(img4.copy(), corners4, ids4)
    
    corners_list = [None] * 4
    for i, (frame, K, R, T) in enumerate(zip([img1, img2, img3, img4], [K0, K1, K2, K3], [R0, R1, R2, R3], [T0, T1, T2, T3])):
        if frame is None:
            continue
        
        corners, ids, _ = arucoDetector.detectMarkers(frame)

        if ids is not None and 0 in ids:
            idx = np.where(ids == 0)[0][0]  #Filtrando para sómente o indice 0
            c = corners[idx][0]
            cx, cy = np.mean(c, axis=0)  #Meio do marcador
            corners_list[i] = (cx, cy) #Lista com os 4

    X = reconstruct_3d_position(corners_list, [K0, K1, K2, K3], [R0, R1, R2, R3], [T0, T1, T2, T3]) #Função de reconstrução

    print(f"Posição 3D estimada: {X}\n")
    X_hist = np.vstack((X_hist,X))
    
    trajetoria.set_data(X_hist[:, 0], X_hist[:, 1])
    trajetoria.set_3d_properties(X_hist[:, 2])
    plt.draw()
    plt.pause(0.005)

    top_row = np.hstack((frame_markers1, frame_markers2)) #Juntar as imagens
    bottom_row = np.hstack((frame_markers3, frame_markers4))  
    img = np.vstack((top_row, bottom_row))
    
    img = cv2.resize(img, (1280,720)) # Reduz o tamanho para ser visivel em uma tela 
    cv2.imshow('output', img)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()