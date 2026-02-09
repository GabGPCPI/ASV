import cv2 as cv
import numpy as np
from pupil_apriltags import Detector
import math
import sys
import time
import traceback
from threading import Thread # Para diagnósticos detalhados de erro

#1. Dicionário das tags, seu tamanho e id
# Dicionário que mapeia ID -> Tamanho do lado preto (em metros)
TAG_CONFIG = {
    # ID: Tamanho (m)
    8: 0.11,      # a minha grande
    9: 0.09,      # a minha pequena
    'DEFAULT': 0.10 # Valor médio se o ID não for reconhecido
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

#2. Classe de Threading (talvez aumente o fps da gravação)
class CameraStream:
    '''
    Lê frames da câmera em uma thread separada para não "travar" o processamento principal
    '''

    def __init__(self, src=0, width=640, height=480):
        self.src = src
        self.width = width
        self.height = height
        
        #Inicia a câmera
        self.stream = cv.VideoCapture(self.src, cv.CAP_DSHOW)
        self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

        #Lê o primeiro frame para garantir que funciona
        (self.grabbed, self.frame) = self.stream.read()

        #Variável de controle
        self.stopped = False
        self.reconnecting = False
        self.thread = None 
    
    def start(self):
        # Guardamos a thread numa variável 'self.thread' para poder esperar ela depois
        self.thread = Thread(target=self.update, args=(), daemon=True)
        self.thread.start()
        return self
    
    def update(self):
        while True:
            if self.stopped:
                if self.stream.isOpened():
                    self.stream.release()
                    print("[Thread] câmera liberada com sucesso")
                return
            
            #Reconexão
            if self.reconnecting:
                try:
                    self.stream = cv.VideoCapture(self.src, cv.CAP_DSHOW)
                    self.stream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
                    self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
                    if self.stream.isOpened():
                        self.reconnecting = False
                        print("[Thread] Câmera Reconectada")
                except:
                    pass
                time.sleep(1.0)
                continue

            (grabbed, frame) = self.stream.read()

            if not grabbed:
                if not self.reconnecting:
                    print("Thread falhou na leitura! Tentando reconectar")
                    self.reconnecting = True
                    if self.stream.isOpened():
                        self.stream.release()
            else:
                #Finalmente atualiza com o frame mais novo
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        #Pega o ultimo frame da lista automaticaente
        return self.frame
    
    def stop(self):
        self.stopped = True
        #Aqui espera a thread terminar antes de seguir
        if self.thread is not None:
            self.thread.join()


#3. Funções auxiliares de câmera
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

def setup_camera_manual_e_thread(width_req, height_req):
    '''Lista as câmeras, pede para o usuário escolher com qual iniciar e também inicia o thread'''
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
    print("---------------------------")

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

    #4. Iniciando programa de imagem com o índice escolhido
    print(f"Abrindo câmera com índice {indice_escolhido}...")

    cam_stream = CameraStream(src=indice_escolhido, width=width_req, height=height_req).start()
    time.sleep(1.0)

    frame = cam_stream.read()

    # Proteção caso a câmera falhe em abrir no primeiro frame
    if frame is None:
        print("Erro Crítico: Não foi possível ler o primeiro frame para calibração.")
        sys.exit(1)

    h, w = frame.shape[:2]
    params = get_c920_params(w, h)
    
    return cam_stream, params

    '''
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

    return cap, params'''

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

# --- 5. PROCESSAMENTO (Fossen Notation) ---
def processar_frame_sensor(frame, detector, camera_params):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=TAG_CONFIG['DEFAULT'])

    dados_output = {}

    for tag in tags:
        tid = tag.tag_id
        real_size = TAG_CONFIG.get(tid, TAG_CONFIG['DEFAULT'])

        # Correção de Escala
        scale = real_size / TAG_CONFIG['DEFAULT']
        pose_t = tag.pose_t * scale

        # Rotação (Yaw)
        yaw_rad = math.atan2(tag.pose_R[1, 0], tag.pose_R[0, 0])
        yaw_deg = -math.degrees(yaw_rad) # NED (Horário +)

        # Notação Fossen: eta = [x, y, psi]
        # x = Surge (Frente), y = Sway (Direita)
        x = pose_t[0][0]
        y = pose_t[1][0]
        psi = yaw_deg

        dados_output[tid] = {'x': x, 'y': y, 'psi': psi}

        # HUD
        cx, cy = int(tag.center[0]), int(tag.center[1])
        cv.circle(frame, (cx, cy), 5, (0,0,255), -1)
        cv.putText(frame, f"ID:{tid}", (cx+10, cy-15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv.putText(frame, f"x:{x:.2f}m", (cx+10, cy), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        cv.putText(frame, f"y:{y:.2f}m", (cx+10, cy+15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        cv.putText(frame, f"psi:{psi:.1f}", (cx+10, cy+30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)

    return frame, dados_output

def menu_resolucao():
    """
    Pergunta ao usuário qual resolução deseja utilizar.
    Retorna: (width, height)
    """
    print("\n--- Configuração de Resolução ---")
    print("Escolha a resolução de trabalho:")
    print("  [1]: 640x480 (Padrão - Maior FPS)")
    print("  [2]: 1280x720 (HD - Melhor Detecção)")
    print("  [3]: 1920x1080 (Full HD - Mais Lento)")
    
    while True:
        escolha = input("Opção: ").strip()
        if escolha == '1':
            return 640, 480
        elif escolha == '2':
            return 1280, 720
        elif escolha == '3':
            return 1920, 1080
        else:
            print("Opção inválida. Digite 1, 2 ou 3.")

#6. MAIN LOOP
def main():
    #Finalmente usa o Thread
    #Resolução 
    largura, altura = menu_resolucao()
    #Inicia a câmera
    cam,params = setup_camera_manual_e_thread(largura, altura)
    Detector = iniciar_detector()

    print("\n--- SENSOR GLOBAL (THREADED) ---")
    print(f"Resolução Alvo: {largura}x{altura}")
    print("Pressione 'q' para sair.")

    fps_count = 0
    start_time = time.time()

    try:
        while True:
            #6.1. Pega o frame instantâneo
            frame = cam.read()
            if frame is None: 
                time.sleep(0.001)
                continue 

            #6.2.     
            frame_hud, dados = processar_frame_sensor(frame, Detector, params)

            #6.3. FPS
            fps_count += 1
            if (time.time() - start_time) > 1.0:
                print(f"FPS: {fps_count}")
                fps_count = 0
                start_time = time.time()

            cv.imshow("Visão Fossen", frame_hud)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrompido.")
    finally:
        print("Fechando recursos do programa")
        cam.stop()
        cv.destroyAllWindows()
        print("Programa encerrado com segurança")

if __name__ == "__main__":
    main()