from pupil_apriltags import Detector # <--- Importar o Detector
import numpy as np
import cv2 as cv
import time
import os

def camera_config():
    """
    Mostra um menu interativo para o usuário escolher a resolução e o FPS.
    Retorna (width, height, fps)
    """
    print("--- Configuração da Câmera ---")
    print("Escolha uma predefinição:")
    print("  [1]: 640x480 @ 30 FPS (Padrão OpenCV)")
    print("  [2]: 1920x1080 @ 60 FPS (Alta Performance)")
    print("  [3]: Personalizado")

    while True:
        escolha = input("Digite sua escolha (1, 2 ou 3): ").strip()

        if escolha == "1":
            return 640, 480, 30.0
        elif escolha == "2":
            return 1920, 1080, 60.0
        elif escolha == "3":
            try:
                largura = int(input("Digite a largura (width) desejada: "))
                altura = int(input("Digite a altura (height) desejada: "))
                fps = float(input("Digite o FPS desejado: "))
                if largura > 0 and altura > 0 and fps > 0:
                    if largura > 1920:
                        print("A largura inserida é muito grande para a câmera atual")
                    else:
                        if altura > 1080:
                            print("A altura inserida é muito grande para a câmera atual")
                        else:
                            if fps > 60:
                                print("O FPS inserido é muito alto para a câmera atual")
                            else: 
                                return largura, altura, fps
                else:
                    print("Erro: Entrada inválida. Por favor, digite apenas números positivos.")
            except ValueError:
                print("Erro: Entrada inválida. Por favor, digite apenas números.")
        else:
            print("Opção inválida. Por favor, escolha 1, 2 ou 3.")

def save_path():
    """
    Pergunta ao usuário onde salvar o arquivo de vídeo.
    Retorna o caminho completo do arquivo.
    """
    print("\n--- Configuração de Gravação ---")
    pasta_script = os.path.dirname(os.path.realpath(__file__))
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    nome_base = f"gravacao_{timestamp}.avi"

    caminho_padrao = os.path.join(pasta_script, nome_base) #Caminho da pasta onde esse .py está armazenado

    while True:
        # Pergunta ao usuário se quer usar o caminho padrão
        choice = input(f"Deseja usar o caminho padrão para salvar?\n  (Padrão: {caminho_padrao})\n  Digite (S/N): ").strip().lower()
        
        if choice == 's' or choice == '': # Aceita 's' ou apenas Enter
            return caminho_padrao
        
        elif choice == 'n':
            pasta_custom = input("Digite o caminho completo da *pasta* onde deseja salvar (ex: C:/Meus_Videos): ").strip()
            
            # Verifica se a pasta customizada existe
            if os.path.isdir(pasta_custom):
                return os.path.join(pasta_custom, nome_base)
            else:
                print(f"Erro: A pasta '{pasta_custom}' não existe. Tente novamente.")
        
        else:
            print("Escolha inválida. Digite 's' para sim ou 'n' para não.")

def setup_camera():
    """
    Pede a configuração ao usuário, encontra e abre a câmera, e define as configurações.
    Retorna o objeto 'captura_de_video', (width, height) e fps reais.
    """
    user_width, user_height, user_fps = camera_config()
    captura_de_video = None
    indice_camera = -1
    backend_funcional = None
    backends_possiveis = [("CAP_DSHOW", cv.CAP_DSHOW),("CAP_MSMF", cv.CAP_MSMF),("CAP_ANY (Padrão)", cv.CAP_ANY)] # O primeiro é o nome do backend, o segundo é o id dele

    print("Testando backends possíveis para abrir a câmera ...")
    for indice in range(4): # Tenta índices 0, 1, 2, 3
        for nome_backend, id_backend in backends_possiveis:
            print(f"Tentando abrir com índice: {indice} e nome: {nome_backend}...")
            captura_de_video = cv.VideoCapture(indice, id_backend) # Tenta abrir a câmera aqui
            if captura_de_video.isOpened():
                print(f"SUCESSO! Câmera aberta usando o índice:{indice} e nome:{nome_backend}.")
                indice_camera = indice
                backend_funcional = nome_backend
                break # Sai do loop de backends
            else:
                print(f"Falha ao abrir com índice:{indice} e nome:{nome_backend}.")
                if captura_de_video is not None: 
                    captura_de_video.release()
        if captura_de_video is not None and captura_de_video.isOpened():
            break # Sai do loop de indices
    if not captura_de_video or not captura_de_video.isOpened():
        print("Nenhum backend funcional encontrado. Encerrando o programa.")
        exit()

    print(f"Câmera solicitada: {user_width}x{user_height} e {int(user_fps)} FPS")
    captura_de_video.set(cv.CAP_PROP_FRAME_WIDTH, user_width)
    captura_de_video.set(cv.CAP_PROP_FRAME_HEIGHT, user_height)
    captura_de_video.set(cv.CAP_PROP_FPS, user_fps)
    
    width = int(captura_de_video.get(cv.CAP_PROP_FRAME_WIDTH))
    height = int(captura_de_video.get(cv.CAP_PROP_FRAME_HEIGHT))
    fps = captura_de_video.get(cv.CAP_PROP_FPS)

    print(f"Resolução real da câmera: {width}x{height} e {int(fps)} FPS.")
    print(f"Câmera aberta usando índice:{indice_camera} e backend funcional: {backend_funcional}.")

    # Retorna o objeto da câmera e suas propriedades reais
    return captura_de_video, (width, height), fps

def detector_apriltag():
    """
    Inicializa e retorna o detector de AprilTags.
    """
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
        return detector
    except Exception as e:
        print(f"Erro ao inicializar pupil_apriltags: {e}")
        print("Tentando biblioteca apriltag alternativa...")
        try:
            import apriltag
            options = apriltag.DetectorOptions(families='tag36h11')
            detector = apriltag.Detector(options)
            print("Usando detector apriltag.")
            return detector
        except Exception as e2:
            print(f"Erro ao inicializar apriltag: {e2}")
            print("Nenhum detector de AprilTag funcional encontrado. Encerrando.")
            exit()

def armazenar_video(nome_arquivo, fps, resolucao):
    """
    Configura e retorna o objeto VideoWriter para salvar o vídeo.
    """
    fourcc = cv.VideoWriter_fourcc(*'XVID') #XVID pois tem maior compatibilidade
    print(f"Salvando vídeo em: {nome_arquivo} com {int(fps)} FPS e resolução {resolucao}")
    return cv.VideoWriter(nome_arquivo, fourcc, fps, resolucao)

def processar_frame(frame, detector):
    """
    Recebe um frame, detecta as tags e desenha as informações nele.
    Retorna o frame processado.
    """
    cinza = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # 1. Converter para tons de cinza PARA DETECÇÃO
    tags_detectadas = detector.detect(cinza) # 2. Detectar as tags na imagem cinza
    print(f"Tags encontradas: {len(tags_detectadas)}") #Número de tags detectadas

    # 3. Processar CADA tag encontrada
    for tag in tags_detectadas:
        try:
            # Extrair informações
            tag_id = tag.tag_id
            centros_arbs = [(int(tag.center[0]), int(tag.center[1]))]
            cantos_arbs = [np.array(tag.corners, dtype=np.int32)]

            # --- Desenhar na imagem COLORIDA ('frame') para visualização ---
            for canto_arb in cantos_arbs:
                cv.polylines(frame, [canto_arb], True, (0, 255, 0), 2) # Contorno verde
            for centro_arb in centros_arbs:
                cv.circle(frame, centro_arb, 5, (0, 0, 255), -1) # Círculo vermelho no centro
                cv.putText(frame, str(tag_id), (centro_arb[0]+10, centro_arb[1]-10), # Coordenadas do centro do arb
                            cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        except Exception as e_tag:
            print(f"Erro ao processar tag: {e_tag}")

    return frame

def releases(captura_de_video, grava_e_salva_video):
    """
    Libera todos os recursos e fecha as janelas.
    """
    print("Liberando a câmera, salvando o video e fechando as janelas de video")
    captura_de_video.release()
    grava_e_salva_video.release()
    cv.destroyAllWindows()

def main():
    # --- 1. FASE DE CONFIGURAÇÃO ---
    cap, resolucao, fps = setup_camera()
    detector = detector_apriltag()
    nome_arquivo_final = save_path()
    video_writer = armazenar_video(nome_arquivo_final, fps, resolucao)

    print("\nIniciando loop principal... Pressione 'e' na gravação para sair.")

    # --- 2. FASE DE EXECUÇÃO (LOOP PRINCIPAL) ---
    while True:
        reticulado, frame = cap.read() #Lê um frame da câmera. 'reticulado' será True se a leitura foi bem-sucedida.
        if not reticulado:
            print("Houve um erro ao capturar o frame, encerrando o programa.")
            break

        frame_processado = processar_frame(frame, detector) #processa o frame

        video_writer.write(frame_processado) #salva o video

        cv.imshow("Minha Câmera", frame_processado) #Mostra o video após o processamento na tela

        # Verifica a tecla de saída
        if cv.waitKey(1) & 0xFF == ord('e'): #1ms de espera pra olhar denovo
            print("Tecla 'e' - a saída do programa foi ativada")
            break

    releases(cap, video_writer)

if __name__ == "__main__":
    main()