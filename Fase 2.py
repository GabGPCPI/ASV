import cv2 as cv
import numpy as np
from pupil_apriltags import Detector
import math
import sys
import time
import traceback # Para diagnósticos detalhados de erro

#1. Dicionário das tags, seu tamanho e id
# Dicionário que mapeia ID -> Tamanho do lado preto (em metros)
TAG_CONFIG = {
    # ID: Tamanho (m)
    8: 0.11,      # a minha grande
    9: 0.09,      # a minha pequena
    'DEFAULT': 0.16 # Valor de segurança se o ID não for reconhecido
}

#2. Configuração da parte óptica (Logitech C920 apenas)
def get_c920_params(width, height):
    '''
    Retorna a matriz intrínseca da câmera [fx, fy, cx, cy] estimada
    Isso evita que a distância fique errada ao trocar de 640x480 para 1280x720
    '''

    #No  Reddit vi que o FOV da câmera é ~78 graus
    if width == 640 and height == 480:
        return [620, 620, 320, 240] #Foco ~620px
    elif width == 1280 and height == 720: 
        return [1240, 1240, 640, 360] #Foco ~1240px
    elif width == 1920 and height == 1080: 
        return [1860, 1860, 960, 540] #Foco ~1860px
    else:
        # Aproximação linear para outras resoluções
        approx_f = width * 0.96
        return [approx_f, approx_f, width/2, height/2]
    
#2. Configuração e seleção da câmera
def listar_cameras_disponiveis():
    '''
    Testa os índices de câmera e vê qual responde
    Retorna uma lista das funcionais
    '''

    print("Buscando os índices da câmera")
    indices_funcionais = []

    #Testa só até o índice 3 (0, 1, 2, 3)
    for index in range (4):
        #CAP_DSHOW é mais rápido no Windows. Linux usa-se CAP_V4L2.
        cap = cv.VideoCapture(index, cv.CAP_DSHOW)

        if cap.isOpened():
            #Tenta ler imagem para garantir
            ret, _ = cap.read()
            if ret:
                indices_funcionais.append(index)
            cap.release()
    return indices_funcionais

def setup_camera_manual(width_req, height_req, fps_req):
    '''Lista as câmeras e pede para o usuário escolher'''
    print(f"Iniciando setup da câmera...")

    #1. Lista de câmeras
    indices = listar_cameras_disponiveis()

    if not indices:
        print("Erro, não foram detectadas câmeras no sistema, verifique a conexão USB")
        sys.exit(1)

    #2. Menu de escolha
    print("\n Câmeras disponíveis:")
    for idx in indices:
        print(f"   [{idx}]: Câmera índice {idx}")
    print("   ---------------------------")

    indice_escolhido = -1

    if len(indices) == 1:
        print(f"Apenas uma câmera encontrada (índice {indices})")
        indice_escolhido = indices[0]
    else: 
        while True: 
            try: 
                entrada = input(f"Digite o índice desejado ({indices}): ").strip()
                escolha = int(entrada)
                if escolha in indices:
                    indice_escolhido = escolha
                    break
                else: 
                    print("Índice inválido. Insira um índice válido")
            except ValueError:
                print("Digite apenas números.")

    #3. Iniciando programa de imagem com o índice escolhido
    print(f"Abrindo câmera com índice {indice_escolhido}...")
    cap = cv.VideoCapture(indice_escolhido, cv.CAP_DSHOW)

    #Configura resolução
    cap.set(cv.CAP_PROP_FRAME_WIDTH, width_req)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, height_req)
    cap.set(cv.CAP_PROP_FPS, fps_req)

    #Leitura real do Hardware
    real_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    print(f"---> Câmera ativa: {real_w}x{real_h}")

    # Ajuste de Lente (get_c920_params é otimizado para C920 (a webcam q o maicom me emprestou), 
    # mas tem um fallback genérico para outras câmeras)
    params = get_c920_params(real_w, real_h)

    return cap, params

def iniciar_detector(): 
    '''Inicia o AprilTag com tratamento de erro.'''
    try: 
        print("Carregando Detector AprilTag")
        detector = Detector(
            families = "tag36h11",
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        return detector
    except Exception as e:
        print("\n Erro: Não foi possivel carregar 'pupil_apriltags")
        traceback.print_exc()
        sys.exit(1)

#SETUP
if __name__ == "__main__":
    # Teste de seleção manual
    # Vamos pedir 640x480 para teste rápido
    cap, params = setup_camera_manual(640, 480, 30)
    det = iniciar_detector()
    
    print("\nTeste de Câmera OK.")
    print("Pressione 'q' na janela (se houvesse) para sair.")
    
    # Solta a câmera para não travar
    cap.release()