from pupil_apriltags import Detector # <--- Importar o Detector
import numpy as np
import cv2 as cv
import time
import os
import math 
import socket

# --- CONSTANTES DE CONTROLE ---
# Ganhos do Controlador PD de Guinada (Yaw)
# Estes valores são "chutados" e precisam ser ajustados (calibrados) na prática.
KP_YAW = 40.0  # Ganho Proporcional (Quão forte ele reage ao erro atual)
KV_YAW = 1.0   # Ganho Derivativo (Quão forte ele amortece a rotação)
CMD_SCALE_FWD = 2.0 # Escala para a força de avanço
CMD_MAX = 999       # Limite máximo dos comandos

# --- CONSTANTES DE REDE ---
# Ajustar conforme necessário
ROBOT_IP = "192.168.0.83" # Exemplo: Substitua pelo IP real do robô
ROBOT_PORT = 2222         # Porta padrão usada no Arduino

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

def get_user_alvo(resolucao):
    """
    Mostra um menu interativo para o usuário escolher o alvo (target).
    Recebe a tupla de resolucao (width, height) para calcular os pontos.
    Retorna a tupla (x, y) do alvo escolhido.
    """
    width, height = resolucao

    # Define os alvos predefinidos com base na resolução
    alvos_predefinidos = {
        '1': [(width // 2, height // 2)],       # Centro
        '2': [(25, 25)],                          # Canto Superior Esquerdo (-, -)
        '3': [(width - 25, 25)],                  # Canto Superior Direito (+, -)
        '4': [(25, height - 25)],                 # Canto Inferior Esquerdo (-, +)
        '5': [(width - 25, height - 25)],         # Canto Inferior Direito (+, +)
        '6': [(width - 25, 25),                 # Canto Superior Direito
                (width - 25, height - 25),      # Canto Inferior Direito
                (25, height - 25),              # Canto Inferior Esquerdo
                (25, 25)],                      # Canto Superior Esquerdo
                # define uma trajetoria quadrada
    }

    print("\n--- Configuração do Alvo ---")
    print(f"Resolução detectada: {width}x{height}")
    print("Escolha um alvo predefinido:")
    print("  [1] Alvo único: Centro da Tela")
    print("  [2] Alvo único: Canto Superior Esquerdo")
    print("  [3] Alvo único: Canto Superior Direito")
    print("  [4] Alvo único: Canto Inferior Esquerdo")
    print("  [5] Alvo único: Canto Inferior Direito")
    print("  [6] Trajetória: Quadrado (4 cantos)")
    print("  [7] Trajetória: Personalizado (digitar X, Y)")

    while True:
        choice = input("Digite sua escolha (1-7): ").strip()
        
        if choice in alvos_predefinidos:
            return alvos_predefinidos[choice]
        
        elif choice == '7':
            lista_waypoints_custom = []
            print("Digite as coordenadas (x,y) de cada waypoint. Para encerrar o input de alvo escreva 'fim'.")
            while True:
                try:
                    x_str = input(f" Digite o X do alvo {len(lista_waypoints_custom) + 1} (ou 'fim' para encerrar): ").strip().lower()
                    if x_str == 'fim':
                        if len(lista_waypoints_custom) > 0: #tem pelo menos um alvo
                            return lista_waypoints_custom
                        else:
                            print("Nenhum alvo foi inserido. Encerrando.")
                            continue
                    y_str = input(f"  Digite o Y do alvo {len(lista_waypoints_custom) + 1}: ").strip()   

                    x = int(x_str)
                    y = int(y_str)

                    if 0 <= x <= width and 0 <= y <= height:
                         lista_waypoints_custom.append((x, y))
                         print(f"Alvo {len(lista_waypoints_custom)}: ({x}, {y}) adicionado.")
                    else:
                        print(f"Erro: Coordenadas fora dos limites da tela (0-{width}, 0-{height}).")
                except ValueError:
                    print("Erro: Entrada inválida. Por favor, digite apenas números.")
        else:
            print("Escolha inválida. Tente novamente.") 

def calcular_vetor_direção(pos_arb, pos_alvo):
    """
    Calcula o vetor de direção unitário do robô para o alvo.
    Retorna um vetor numpy [dx, dy] normalizado.
    """
    # Converte para numpy arrays para facilitar a matemática
    pos_arb = np.array(pos_arb)
    pos_alvo = np.array(pos_alvo)

    # 1. Calcula o vetor de direção (Alvo - Robô)
    vetor_direcao = pos_alvo - pos_arb

    # 2. Calcula a magnitude (distância) do vetor
    distancia = np.linalg.norm(vetor_direcao) #calcula a norma de vetores e matrizes no Python

    # 3. Medida de segurança para evitar divisão por zero
    if distancia < 1e-6: # Se estiver muito perto (ou em cima)
        return np.array([0.0, 0.0]), 0.0 # Retorna vetor nulo -> o arb já atingiu o alvo

    # 4. Normaliza o vetor (divide pela distância) para obter o vetor unitário
    vetor_unitario = vetor_direcao / distancia 
    
    return vetor_unitario, distancia

def calcular_angulo_menor(angulo_atual_graus, angulo_alvo_graus):
    """
    Calcula o caminho angular mais curto entre dois ângulos (em graus).
    Retorna a diferença em graus (-180 a +180).
    """
    diff = (angulo_alvo_graus - angulo_atual_graus) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff

def calcular_propulsao(estado_arb, pos_arb, yaw_robo_rad, pos_alvo, dt):
    """
    Calcula os comandos de propulsão (giro e avanço) para um robô.
    Implementa um controlador PD para guinada (yaw) e P para avanço.
    """
    # --- 1. CÁLCULO DO CONTROLADOR DE GUINADA (YAW PD-Controller) ---
    # a. Calcular o ângulo desejado (para onde o robô deve apontar)
    vetor_dir, distancia = calcular_vetor_direção(pos_arb, pos_alvo)
    yaw_alvo_rad = math.atan2(vetor_dir[1], vetor_dir[0])

    # b. Calcular o erro Proporcional (P)
    yaw_robo_graus = math.degrees(yaw_robo_rad)
    yaw_alvo_graus = math.degrees(yaw_alvo_rad)
    erro_yaw_graus = calcular_angulo_menor(yaw_robo_graus, yaw_alvo_graus)
    erro_yaw_rad = math.radians(erro_yaw_graus)

    # c. Calcular a velocidade angular (D)
    taxa_yaw = 0.0
    if dt > 1e-6: #para não haver divisão por zero
        diff_yaw = calcular_angulo_menor(estado_arb['yaw_antigo_graus'], yaw_robo_graus)
        taxa_yaw = diff_yaw / dt #sai em graus/seg

    # Salva o estado atual para o próximo ciclo
    estado_arb['yaw_antigo_graus'] = yaw_robo_graus

    # d. Aplicar a Lei de Controle PD (retirada da dissertação de mestrado do emerson)
    # (KP * Erro P) - (KV * Erro D)
    forca_giro = (KP_YAW * erro_yaw_rad) - (KV_YAW * math.radians(taxa_yaw)) # Converte-se taxa_yaw para rad/s

    # e. Formatar o comando de giro 
    # Deve-se adicionar uma força de giro mínima para vencer a inércia
    FpsiProp = forca_giro
    if FpsiProp > 0:
        FpsiProp = 0.7 * abs(FpsiProp) + 400 # 400 para vencer a inércia, rotyação positiva
    elif FpsiProp < 0:
        FpsiProp = -(0.7 * abs(FpsiProp) + 400) # rotação negativa

    mag_turn = int(min(max(int(abs(FpsiProp)), 0), CMD_MAX)) #magnitude da rotação
    sign = '-' if FpsiProp >= 0 else '+' #sinal da rotação
    cmd_giro = sign + str(mag_turn).zfill(4) #comando de giro formatado (ex: "-0450")

    # --- 2. CÁLCULO DO CONTROLADOR DE AVANÇO (P-Controller) ---
    # A força de avanço é proporcional a distância ao alvo

    # Se o robô tiver status 'concluido', a força de avanço é 0
    if estado_arb['status'] == 'concluido':
        forca_avanco = 0.0
    else:
        forca_avanco = int(min(max(distancia * CMD_SCALE_FWD, 0), CMD_MAX)) #força de avanço
    
    mag_fwd = int(min(max(int(abs(forca_avanco)), 0), CMD_MAX)) #magnitude da força de avanço
    cmd_avanco = str(mag_fwd).zfill(4) #comando de avanço formatado (ex: "0300")

    return cmd_giro, cmd_avanco, erro_yaw_graus, taxa_yaw

def setup_network():
    """
    Configura o socket UDP para comunicação com o robô.
    Retorna o socket configurado.
    """
    try:
        sock = socket.socket(socket.AD_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0) #time out
        print (f"Rede configurada com sucesso! Arb: {ROBOT_IP}:{ROBOT_PORT}")
        return sock
    except Exception as e:
        print(f"Erro ao configurar rede: {e}")
        return None

def processar_frame(frame, detector, lista_waypoints, arb_states, resolucao, fps, dt, sock):
    """
    Recebe um frame, detecta as tags e suas informações, desenha as informações nele, calcula a lógica e envia comandos.
    Retorna o frame processado.
    """

    # --- HUD ---
    # Define as propriedades do texto
    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    cor_texto = (255, 255, 255) # Branco
    espessura = 1
    y_pos = 30 # Posição Y inicial no canto superior esquerdo
    y_incremento = 25 # Espaçamento entre linhas 
    # Distância para considerar que um waypoint foi "alcançado"
    limite_alcance = 25 #25 pixels (seria o tamanho do robo visto de cima/2)
    

    # HUD 1. Desenha a Resolução e o FPS=
    texto_res = f"Resolucao: {resolucao[0]}x{resolucao[1]} e FPS: {int(fps)} FPS"
    cv.putText(frame, texto_res, (10, y_pos), font, font_scale, cor_texto, espessura)
    y_pos += y_incremento # Incrementa Y para a próxima linha

    # HUD 2. Desenha a o numero de Waypoints Alvo
    texto_trajetoria = f"Trajetoria: {len(lista_waypoints)} Waypoint(s)"
    cv.putText(frame, texto_trajetoria, (10, y_pos), font, font_scale, cor_texto, espessura)
    y_pos += y_incremento # Incrementa Y para a próxima linha

    # Desenha os waypoints na tela
    for i , ponto in enumerate(lista_waypoints):
        cv.circle(frame, ponto, 10, (0, 255, 255), 2) #circulo amarelo no alvo i 
        cv.putText(frame, str(i), (ponto[0] + 15, ponto [1] + 5), font, 0.6, (0, 255, 255), 2)
    # --- FIM DO BLOCO HUD ESTÁTICO ---

    '''
    cv.circle(frame, alvo_int, 15, (0, 255, 255), 2) # Círculo Amarelo no alvo
    #desenha um + no alvo
    cv.line(frame, (alvo_int[0] - 10, alvo_int[1]), (alvo_int[0] + 10, alvo_int[1]), (0, 255, 255), 2)
    cv.line(frame, (alvo_int[0], alvo_int[1] - 10), (alvo_int[0], alvo_int[1] + 10), (0, 255, 255), 2)
    '''

    cinza = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # 1. Converter para tons de cinza PARA DETECÇÃO
    tags_detectadas = detector.detect(cinza) # 2. Detectar as tags na imagem cinza
    '''print(f"ARBs encontrados: {len(tags_detectadas)}") #Número de tags detectadas'''

    # --- HUD DINÂMICO ---
    # HUD 3. Processar CADA tag encontrada (LÓGICA MULTI-ROBÔ)
    # Esta informação agora é por arb, devo mover para dentro do loop de tags
    y_pos_tags = y_pos + y_incremento # Posição Y separada para o HUD das tags
    texto_hud_tags = f"ARB's Detectados: {len(tags_detectadas)}"
    cv.putText(frame, texto_hud_tags, (10, y_pos), font, font_scale, cor_texto, espessura) #o número de tags aparece na saída, não no terminal
    y_pos_tags += y_incremento

    for tag in tags_detectadas:
        try:
            # --- A. IDENTIFICAR O ROBÔ ---
            tag_id = tag.tag_id
            centro_arb_n = (int(tag.center[0]), int(tag.center[1]))
            #cantos_arb_n = np.array(tag.corners, dtype=np.int32)
            cantos = tag.corners # cantos[0]=ptA, cantos[1]=ptB, cantos[2]=ptC, cantos[3]=ptD

            # --- B. CALCULAR ÂNGULO ATUAL DO ROBÔ ---
            #descobrir qual é a "frente" do robo ou posicionar de acordo com esse código
            # Ângulo do vetor do centro (cX, cY) para o meio da borda frontal (ptB, ptC)
            meio_frente_x = (cantos[1][0] + cantos[2][0]) / 2.0
            meio_frente_y = (cantos[1][1] + cantos[2][1]) / 2.0
            yaw_robo_rad = math.atan2(meio_frente_y - centro_arb_n[1], meio_frente_x - centro_arb_n[0])

            # --- C. GERENCIAR O ESTADO DO ROBÔ ---
            if tag_id not in arb_states:
                arb_states[tag_id] = {'waypoint_indice': 0, 'status': 'ativo', 'yaw_antigo_graus': math.degrees(yaw_robo_rad), 'tempo_anterior': time.time()}  # Se for um robô novo, inicializa seu estado no dicionário

            # Pega o estado atual deste robô
            estado_arb = arb_states[tag_id]

            #print(f"centros arb: {centro_arb_n}")
            #print(f"cantos arb: {canto_arb_n}")
            print(f"estado_arb: {estado_arb} DENTRO DO FRAME_PROCESSOR")

            # Se o robô já concluiu a trajetoria, apenas o desenha e pula para a próxima tag
            if estado_arb['status'] == 'concluido':
                cv.polylines(frame, [cantos.astype(np.int32)], True, (255, 255, 255), 1) #contorno branco para robo inativo (missao concluida) 
                #cantos.astype(np.int32) também pode ser cantos_arb_n se descomentar ele'
                cv.putText(frame, f"ARB de ID {tag_id} (concluiu a trajetoria)", (centro_arb_n[0]+10, centro_arb_n[1]-10), 
                            font, 0.5, (255, 255, 255), 1)
                
                if sock:
                    msg_parada = "{-00000000}"
                    try:
                        #UDP: Desconectar a linha quando definit o IP e Porta dos robos
                        #sock.sendto(msg_parada.encode(), (ROBOT_IP, ROBOT_PORT))
                        pass
                    except Exception:
                        pass
                continue # Pula para a próxima tag

            # --- D. DETERMINAR O ALVO ATUAL DO ROBÔ ---
            indice_alvo_atual = estado_arb['waypoint_indice']
            alvo_atual = lista_waypoints[indice_alvo_atual]

            # --- E. CALCULAR PROPULSÃO ---
            cmd_giro, cmd_avanco, erro_yaw_graus, taxa_yaw = calcular_propulsao(estado_arb, centro_arb_n, yaw_robo_rad, alvo_atual, dt)

            # --- F. ENVIAR COMANDO UDP ---

            # Monta a mensagem no formato: {+GIROAVANCO} ex: {-04500300}
            msg_comando = f"{{{cmd_giro}{cmd_avanco}}}"

            if sock:
                try:
                    # IMPORTANTE: Preciso ajustar a lógica para enviar para IPs diferentes (cada robo tem seu próprio IP)
                    # baseados no tag_id. Por enquanto, envio para o robô 0 -> IP padrão definido por mim.
                    #sock.sendto(msg_comando.encode(), (ROBOT_IP, ROBOT_PORT))
                    pass
                except Exception as e_udp:
                    print(f"Erro ao enviar comando UDP: {e_udp}")

            # --- F. DESENHAR O ROBÔ E SEU ALVO ---
            cv.polylines(frame, [cantos.astype(np.int32)], True, (0, 255, 0), 2) # Contorno verde (ativo)
            cv.circle(frame, centro_arb_n, 5, (0, 0, 255), -1) # Círculo vermelho no centro
            cv.putText(frame, str(tag_id), (centro_arb_n[0]+10, centro_arb_n[1]-10), font, 0.7, (0, 0, 255), 2)
            
            # Desenha uma linha para mostrar a orientação atual do robô
            ponta_yaw_x = int(centro_arb_n[0] + 30 * math.cos(yaw_robo_rad))
            ponta_yaw_y = int(centro_arb_n[1] + 30 * math.sin(yaw_robo_rad))
            cv.line(frame, centro_arb_n, (ponta_yaw_x, ponta_yaw_y), (0, 0, 255), 2) # Linha Vermelha de Orientação

            '''# --- G. CALCULAR VETOR E DISTÂNCIA --- 
            vetor_dir, distancia = calcular_vetor_direção(centro_arb_n, alvo_atual)'''
           
            cv.line(frame, centro_arb_n, alvo_atual, (255, 100, 100), 2)  # Desenha a linha de direção (visualização) # Linha azul-claro

            # --- H. ATUALIZAR O HUD COM DADOS DESTE ARB ---

            texto_tag_pos = f"ARB de ID: {tag_id} esta na posicao: ({centro_arb_n[0], centro_arb_n[1]})"
            cv.putText(frame, texto_tag_pos, (10, y_pos_tags), font, font_scale, cor_texto, espessura)
            y_pos_tags += y_incremento

            texto_tag_alvo = f"ARB de ID:{tag_id} tem alvo com indice: {indice_alvo_atual} de coordenadas: ({alvo_atual[0]}, {alvo_atual[1]})"
            cv.putText(frame, texto_tag_alvo, (10, y_pos_tags), font, font_scale, (0, 255, 255), espessura)
            y_pos_tags += y_incremento

            '''#Colocar uma nova linha de vetor direção
            texto_tag_vet = f"ARB de ID: {tag_id} precisa de vetor direção: [{vetor_dir[0]:.2f}, {vetor_dir[1]:.2f} pro alvo de indice: {indice_alvo_atual}]"
            cv.putText(frame, texto_tag_vet, (10, y_pos), font, font_scale, cor_texto, espessura)
            y_pos_tags += y_incremento'''

            texto_tag_erro = f"Tag {tag_id} Erro Yaw: {erro_yaw_graus:.1f} graus"
            cv.putText(frame, texto_tag_erro, (10, y_pos_tags), font, font_scale, cor_texto, espessura)
            y_pos_tags += y_incremento

            texto_comandos = f"Tag {tag_id} CMD: {{{cmd_giro}{cmd_avanco}}}"
            cv.putText(frame, texto_comandos, (10, y_pos_tags), font, font_scale, (0, 165, 255), espessura) # Laranja
            y_pos_tags += y_incremento

            # --- I. LÓGICA DE TROCA DE WAYPOINT (SWITCHING) ---

            distancia = np.linalg.norm(np.array(alvo_atual) - np.array(centro_arb_n))
            if distancia < limite_alcance:
                if indice_alvo_atual < len(lista_waypoints) - 1: # Verifica se NÃO é o último waypoint da lista
                    estado_arb['waypoint_indice'] += 1 #Se não for o último waypoint, atualizo o seu waypoint_indice para o proximo alvo para que entre no loop denovo
                    print(f"ARB de ID: {tag_id} alcançou o waypoint de indice {indice_alvo_atual}! Definindo seu proximo alvo como: {estado_arb['waypoint_indice']}") 
                else:
                    # Chegou ao último waypoint
                    estado_arb['status'] = 'concluido' #vai para o estado concluido e fica com contorno branco 
                    print(f"ARB de ID: {tag_id} completou a trajetória!")
        except Exception as e_tag:
            print(f"Erro ao processar tag: {e_tag}")
            import traceback
            traceback.print_exc() # Imprime o erro detalhado

    return frame

def releases(captura_de_video, grava_e_salva_video,sock):
    """
    Libera todos os recursos e fecha as janelas.
    """
    print("Liberando a câmera, salvando o video e fechando as janelas de video")
    captura_de_video.release()
    grava_e_salva_video.release()
    if sock:
        sock.close()
    cv.destroyAllWindows()

def main():
    # --- 1. FASE DE CONFIGURAÇÃO ---
    cap, resolucao, fps = setup_camera()
    detector = detector_apriltag()
    nome_arquivo_final = save_path()

    video_writer = armazenar_video(nome_arquivo_final, fps, resolucao)
    if not video_writer.isOpened():
        print("ALERTA: Não foi possível criar o arquivo de vídeo. Verifique o codec XVID e as permissões da pasta.")
        # O programa pode continuar, mas não gravará

    # --- DEFINE OS ALVOS ---
    lista_waypoints = get_user_alvo(resolucao) 
    print(f"Trajetória definida com {len(lista_waypoints)} pontos.")

    # --- CONFIGURAÇÃO DE REDE ---
    udp_socket = setup_network()

    # --- DICIONÁRIO DE ESTADO DOS ARBS ---
    arb_states = {}  # A chave será o tag_id, o valor será outro dicionário com seu estado
    print(f"Estado dos arbs: {arb_states} DENTRO DA MAIN")

    print("\nIniciando loop principal... Pressione 'e' na gravação para sair.")

    #Inicializa o timer para o cálculo do 'dt'
    tempo_anterior = time.time()

    # --- 2. FASE DE EXECUÇÃO (LOOP PRINCIPAL) ---
    while True:
        agora = time.time()
        dt = agora - tempo_anterior
        tempo_anterior = agora
        if dt <= 1e-6:
            dt = 1e-6 #para evitar divisão por zero caso o loop vá mt rápido
        
        reticulado, frame = cap.read() #Lê um frame da câmera. 'reticulado' será True se a leitura foi bem-sucedida.
        if not reticulado:
            print("Houve um erro ao capturar o frame, encerrando o programa.")
            break

        frame_processado = processar_frame(frame, detector, lista_waypoints, arb_states, resolucao, fps, dt, udp_socket) #processa o frame

        if video_writer.isOpened():    
            video_writer.write(frame_processado) #salva o video

        cv.imshow("Minha Câmera", frame_processado) #Mostra o video após o processamento na tela

        # Verifica a tecla de saída
        if cv.waitKey(1) & 0xFF == ord('e'): #1ms de espera pra olhar denovo
            print("Tecla 'e' - a saída do programa foi ativada")
            break

    releases(cap, video_writer, udp_socket)

if __name__ == "__main__":
    main()