from pupil_apriltags import Detector
import numpy as np
import cv2 as cv
import time

captura_de_video = None
indice_camera = -1 # Para sabermos qual índice funcionou
backends_tentar = [cv.CAP_ANY] # Começa com o padrão
# Adiciona backends específicos do Windows se disponíveis (ignora se der erro)
try:
    backends_tentar.extend([cv.CAP_DSHOW, cv.CAP_MSMF])
except AttributeError:
    pass # Ignora se os backends não existirem na sua versão do OpenCV

print("Procurando por câmeras...")

# Tenta índices 0, 1, 2, 3
for indice in range(4):
    print(f"--- Tentando índice {indice} ---")
    for backend in backends_tentar:
        backend_nome = f" (Backend: {backend})" if backend != cv.CAP_ANY else " (Backend: Padrão)"
        print(f"Tentando abrir com índice {indice}{backend_nome}...")
        captura_de_video = cv.VideoCapture(indice, backend)

        if captura_de_video is not None and captura_de_video.isOpened():
            print(f"SUCESSO! Câmera aberta no índice {indice}{backend_nome}")
            indice_camera = indice
            captura_de_video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            captura_de_video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
            largura = captura_de_video.get(cv.CAP_PROP_FRAME_WIDTH)
            altura = captura_de_video.get(cv.CAP_PROP_FRAME_HEIGHT)
            print(f"Resolução definida para: {int(largura)}x{int(altura)}")
            break
    if captura_de_video is not None and captura_de_video.isOpened():
        break

if captura_de_video is None or not captura_de_video.isOpened():
    print("ERRO FATAL: Nenhuma câmera funcional encontrada.")
    exit()

print(f"\nUsando câmera no índice {indice_camera}. Iniciando captura...")

# --- Bloco de gravação de vídeo (temporariamente desativado para depuração) ---
# fourcc = cv.VideoWriter_fourcc(*'XVID')
# timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
# pasta_destino = 'C:/Users/Gabriel/Desktop/PySwarming/Video salvos/'
# nome_arquivo = f"{pasta_destino}gravacao_{timestamp}.avi"
# print(f"Salvando vídeo em: {nome_arquivo}")
# grava_e_salva_video = cv.VideoWriter(nome_arquivo, fourcc, 30.0, (640,  480))
# --- Fim do bloco de gravação ---

# --- Cria o objeto Detector ---
try:
    detector = Detector(families="tag36h11", nthreads=2, quad_decimate=2.0)
    print("Usando detector pupil_apriltags.")
except Exception as e:
    print(f"Nenhum detector de AprilTag funcional encontrado. Encerrando. Erro: {e}")
    exit()
# --- Fim da criação do Detector ---

print("Iniciando loop principal...")
while captura_de_video.isOpened():
    print("--- Topo do loop ---")

    reticulado, frame = captura_de_video.read()
    if not reticulado:
        print("Houve um erro ao capturar o frame, encerrando o programa.")
        break

    print("Frame lido com sucesso")

    cinza = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    print("Convertido para cinza")

    # CORREÇÃO 1: Remover os colchetes extras []
    tags_detectadas = detector.detect(cinza)
    print(f"Tags encontradas: {len(tags_detectadas)}")

    for tag in tags_detectadas:
        try: # Adicionado try/except para robustez
            tag_id = tag.tag_id
            centro = tag.center
            centro_x = int(centro[0])
            centro_y = int(centro[1])
            cantos = tag.corners

            print(f"  Tag ID: {tag_id} encontrada no centro ({centro_x}, {centro_y})")

            cv.circle(frame, (centro_x, centro_y), 5, (0, 0, 255), -1) # Círculo vermelho

            for canto in cantos:
                canto_int = (int(canto[0]), int(canto[1]))
                cv.circle(frame, canto_int, 2, (0, 255, 0), -1) # Círculos verdes
        except Exception as e_tag:
            print(f"Erro ao processar a tag: {e_tag}")
            continue # Pula para a próxima tag se houver erro

    print("Após processar tags")

    # grava_e_salva_video.write(frame) # Temporariamente desativado

    print("Antes de imshow")
    # CORREÇÃO 2: Exibir o 'frame' colorido para ver os desenhos
    cv.imshow("Minha câmera", frame)
    print("Após imshow, antes de waitKey")

    if cv.waitKey(1) & 0xFF == ord('e'):
        print("Tecla 'e' - a saída do programa foi ativada")
        break

    print("--- Fim do loop ---")

print("Liberando a câmera e fechando as janelas de video")
captura_de_video.release()
# grava_e_salva_video.release() # Temporariamente desativado
cv.destroyAllWindows()