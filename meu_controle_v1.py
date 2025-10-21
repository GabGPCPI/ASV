from pupil_apriltags import Detector # <--- Importar o Detector
import numpy as np
import cv2 as cv
import time

captura_de_video = cv.VideoCapture(0)

#Define a resolução do video
captura_de_video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
captura_de_video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

while not captura_de_video.isOpened():
    print("Falha ao abrir a câmera, tentando novamente em 1 segundo")
    time.sleep(1) #espera 1 seg
    captura_de_video.release() #é uma boa prática liberar a câmera antes de tentar abrir denovo.
    captura_de_video = cv.VideoCapture(0) #tenta reabrir a câmera
    #Define a resolução do video
    captura_de_video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    captura_de_video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

print("Câmera aberta com sucesso")

#cria um objeto para criar o video
#fourcc = cv.VideoWriter_fourcc(*'XVID')

#Exemplo: "2025-10-17_16-55-00" (Ano-Mês-Dia_Hora-Minuto-Segundo)
#timestamp = time.strftime("%Y-%m-%d_%H-%M-%S") #Obter a string de data/hora atual formatada

#pasta_destino = 'C:/Users/Gabriel/Desktop/PySwarming/Video salvos/'

#nome_arquivo = f"{pasta_destino}gravacao_{timestamp}.avi"
#print(f"Salvando vídeo em: {nome_arquivo}") # Informa ao usuário onde o arquivo será salvo

#grava_e_salva_video = cv.VideoWriter(nome_arquivo, fourcc, 30.0, (640,  480))

#identifica a tag
# --- Cria o objeto Detector ---
try:
    detector = Detector(
        families="tag36h11",
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=0,
            decode_sharpening=0.0,
            debug=0)
    print("Usando detector pupil_apriltags.")
except Exception as e:
    print(f"Erro ao inicializar pupil_apriltags: {e}")
    print("Tentando biblioteca apriltag alternativa...")
    try:
        import apriltag
        options = apriltag.DetectorOptions(families='tag36h11')
        detector = apriltag.Detector(options)
        print("Usando detector apriltag.")
    except Exception as e2:
        print(f"Erro ao inicializar apriltag: {e2}")
        print("Nenhum detector de AprilTag funcional encontrado. Encerrando.")
        exit() # Encerra o programa se nenhum detector funcionar

# --- Fim da criação do Detector ---

print("Iniciando loop principal...")
while captura_de_video.isOpened():

    reticulado, frame = captura_de_video.read() #Lê um frame da câmera. 'reticulado' será True se a leitura foi bem-sucedida.

    if not reticulado: #sem leitura de imagem
        print("Houve um erro ao capturar o frame, encerrando o programa.")
        break

    # 1. Converter para tons de cinza PARA DETECÇÃO
    cinza = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #define a cor da janela - tons de cinza

    # 2. Detectar as tags na imagem cinza
    tags_detectadas = [detector.detect(cinza)]
    print(f"Tags encontradas: {len(tags_detectadas)}") # Linha de debug útil
    
    # 3. Processar CADA tag encontrada
    for tag in tags_detectadas:
        # Extrair informações
        tag_id = tag.tag_id
        centro = tag.center # Coordenadas já vêm como (x, y)
        centro_x = int(centro[0])
        centro_y = int(centro[1])
        cantos = tag.corners # Os 4 cantos da tag

        # Imprimir informações no terminal
        print(f"  Tag ID: {tag_id} encontrada no centro ({centro_x}, {centro_y})")

        # --- Desenhar na imagem COLORIDA ('frame') para visualização ---
        # Desenha um círculo no centro
        cv.circle(frame, (centro_x, centro_y), 5, (255, 0, 0), -1) # Círculo azul preenchido

        # Desenha os cantos da tag
        for canto in cantos:
            canto_int = (int(canto[0]), int(canto[1]))
            cv.circle(frame, canto_int, 2, (0, 255, 0), -1) # Círculos verdes preenchidos nos cantos

        # Desenha as linhas entre os cantos --- ? ---
        #pts = np.array(cantos, dtype=np.int32)
        #cv.polylines(frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2) # Contorno azul

    #frame = cv.flip(frame, 0) #deixa o video de ponta cabeça
    #grava_e_salva_video.write(cinza) #salva o video em tons de cinza
    #grava_e_salva_video.write(frame) #salva o video colorido, com os desenhos

    cv.imshow("Minha câmera", cinza) #Mostra o frame na janela na cor cinza

    if cv.waitKey(1) & 0xFF == ord('e'): 
        #Espera 1ms por uma tecla. Se a tecla 'e' for pressionada, encerra o loop.
        print("Tecla 'e' - a saída do programa foi ativada")
        break

print("Liberando a câmera, salvando o video e fechando as janelas de video")
captura_de_video.release()
#grava_e_salva_video.release()
cv.destroyAllWindows()