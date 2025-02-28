import numpy as np
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # necessário para plot 3D



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

# Cria figura e eixo 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plota o ponto (0,0,0) em preto (origem do ambiente)
ax.scatter(0, 0, 0, color='k', s=50)

# Plot dos eixos para cada câmera usando os parâmetros extraídos
plot_camera_axes(R0, T0, ax, scale=0.5, label='Câmera 0')
plot_camera_axes(R1, T1, ax, scale=0.5, label='Câmera 1')
plot_camera_axes(R2, T2, ax, scale=0.5, label='Câmera 2')
plot_camera_axes(R3, T3, ax, scale=0.5, label='Câmera 3')

# Configurações adicionais do gráfico
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Orientação das Câmeras no Ambiente')

plt.show()

