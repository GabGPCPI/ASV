from pupil_apriltags import Detector # <--- Importar o Detector
import numpy as np
import cv2 as cv
import time

backends_possiveis = [("CAP_DSHOW", cv.CAP_DSHOW),("CAP_MSMF", cv.CAP_MSMF),("CAP_ANY (Padrão)", cv.CAP_ANY)]
#o primeiro é o nome do backend, o segundo é o id dele

captura_de_video = None
backend_funcional = None

print("Testando backends possíveis para abrir a câmera ...")

# Tenta índices 0, 1, 2, 3
for indice in range(4):
    for nome_backend, id_backend in backends_possiveis:
        print(f"Tentando abrir com índice {indice}{nome_backend}...")
        captura_de_video = cv.VideoCapture(indice, id_backend)
        if captura_de_video.isOpened():
            print(f"SUCESSO! Câmera aberta usando o índice:{indice} e nome:{nome_backend}.")
            backend_funcional = nome_backend
            break # Sai do loop de backends
        else:
            print(f"Falha ao abrir com índice:{indice} e nome:{nome_backend}.")
            if captura_de_video is not None: 
                captura_de_video.release()
                #boa prática dar o release para a captura de video antes de tentar denovo
                #caso o captura_de_video seja nulo daria erro -> Por isso o if
    if captura_de_video is not None and captura_de_video.isOpened():
        break #sai do loop de índices caso a câmera abra dentro do loop

if not captura_de_video or not captura_de_video.isOpened():
    print("Nenhum backend funcional encontrado. Encerrando o programa.")
    exit()

captura_de_video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
captura_de_video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
print("Resolução da câmera ajustada para 640x480.")

print(f"Câmera aberta usando índice:{indice} e backend funcional: {backend_funcional}. Iniciando captura de video.")

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
    tags_detectadas = detector.detect(cinza) #sai como uma lista de tags já
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