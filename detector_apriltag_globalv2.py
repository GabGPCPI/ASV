from pupil_apriltags import Detector
import numpy as np
import cv2 as cv
import math

# --- 1. BANCO DE DADOS DE TAGS (FÍSICA) ---
# Dicionário que mapeia ID -> Tamanho Real do Quadrado Preto (em Metros)
# ATENÇÃO: Se você não medir isso com régua, o robô vai bater.
TAG_CONFIG = {
    # Exemplo: Robô com tag pequena e Base com tag grande
    0: 0.16,    # Tag do Robô Principal (16cm)
    1: 0.10,    # Tag Menor (10cm)
    2: 0.20,    # Base de Carga (20cm)
    8: 0.11,
    9: 0.067,
    
    'DEFAULT': 0.16 # Tamanho de referência para cálculo inicial
}

def get_c920_params(width, height):
    """
    Retorna a Matriz Intrínseca estimada para Logitech C920
    baseada na resolução escolhida.
    
    Retorna: [fx, fy, cx, cy]
    """
    # A C920 tem um FOV ~78 graus diagonal.
    # Valores empíricos comuns para calibração OpenCV:
    
    if width == 640 and height == 480:
        # Foco aprox 615 a 625 px
        return [620, 620, 320, 240]
    
    elif width == 1280 and height == 720:
        # Foco dobra em relação a 640 (proporcional a largura)
        return [1240, 1240, 640, 360]
    
    elif width == 1920 and height == 1080:
        # Foco triplica
        return [1860, 1860, 960, 540]
        
    else:
        # FÓRMULA DE EMERGÊNCIA (Caso use resolução customizada)
        # Assume-se que fx é aproximadamente igual a largura (para FOV ~60-70 horiz)
        # Para C920, fx é aprox 0.96 * width na proporção 16:9
        approx_f = width * 0.96 
        return [approx_f, approx_f, width/2, height/2]

def camera_config():
    """
    Menu para escolha de resolução.
    """
    print("\n--- Configuração da Câmera (Logitech C920) ---")
    print("Escolha a resolução de trabalho:")
    print("  [1]: 640x480 @ 30 FPS (Padrão - Rápido)")
    print("  [2]: 1280x720 @ 30 FPS (HD - Mais Preciso)")
    print("  [3]: 1920x1080 @ 30 FPS (Full HD - Lento)")
    
    escolha = input("Opção: ").strip()
    if escolha == "2":
        return 1280, 720, 30.0
    elif escolha == "3":
        return 1920, 1080, 30.0
    else:
        return 640, 480, 30.0

def setup_camera():
    width, height, fps_req = camera_config()
    
    # Tenta índices comuns (0, 1) para webcam USB
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv.VideoCapture(1, cv.CAP_DSHOW)
        if not cap.isOpened():
            print("ERRO CRÍTICO: Nenhuma câmera encontrada.")
            exit()
            
    cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv.CAP_PROP_FPS, fps_req)
    
    # Lê o que a câmera realmente entregou (as vezes ela nega o pedido)
    real_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    
    print(f"Câmera iniciada: {real_w}x{real_h}")
    
    # Gera os parâmetros dinamicamente baseado no que a câmera aceitou
    current_params = get_c920_params(real_w, real_h)
    print(f"Parâmetros Intrínsecos Calculados: {current_params}")
    
    return cap, current_params

def detector_apriltag():
    try:
        detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        return detector
    except Exception as e:
        print(f"Erro ao iniciar detector: {e}")
        exit()

def processar_frame_sensor(frame, detector, camera_params):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # 1. DETECÇÃO COM REFERÊNCIA PADRÃO
    # O detector calcula a pose achando que tudo tem o tamanho DEFAULT
    ref_size = TAG_CONFIG['DEFAULT']
    
    tags = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=ref_size
    )

    dados_output = {}

    for tag in tags:
        # 2. IDENTIFICAÇÃO DO TAMANHO REAL
        if tag.tag_id in TAG_CONFIG:
            real_size = TAG_CONFIG[tag.tag_id]
        else:
            real_size = ref_size
            
        # 3. CORREÇÃO DE ESCALA (Linear)
        # Se a tag é metade do tamanho da referência, ela está na metade da distância calculada.
        scale_factor = real_size / ref_size
        
        # Corrige Translação (X, Y, Z)
        pose_t_raw = tag.pose_t
        pose_t_real = pose_t_raw * scale_factor
        
        # 4. CORREÇÃO DE ROTAÇÃO (NED)
        pose_R = tag.pose_R
        yaw_rad = math.atan2(pose_R[1, 0], pose_R[0, 0])
        
        # INVERSÃO CRÍTICA: Convertendo trigonométrico (CCW+) para Naval (CW+)
        yaw_deg = -math.degrees(yaw_rad)

        # Extração Final de Dados
        x_m = pose_t_real[0][0]
        y_m = pose_t_real[1][0]
        id_robo = tag.tag_id
        
        dados_output[id_robo] = {'x': x_m, 'y': y_m, 'yaw': yaw_deg}

        # --- VISUALIZAÇÃO (HUD) ---
        cx, cy = int(tag.center[0]), int(tag.center[1])
        
        # Círculo e ID
        cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv.putText(frame, f"ID:{id_robo}", (cx+10, cy-15), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Dados Métricos
        cv.putText(frame, f"X:{x_m:.2f}m", (cx+10, cy+5), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv.putText(frame, f"Y:{y_m:.2f}m", (cx+10, cy+20), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv.putText(frame, f"Yaw:{yaw_deg:.1f}", (cx+10, cy+35), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                   
        # Indicador de Tamanho usado (Debug)
        cv.putText(frame, f"Sz:{real_size}m", (cx+10, cy+50), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Seta de Orientação (Visual = Original, sem inversão NED para desenhar correto na tela)
        # Nota: O OpenCV desenha num plano onde Y é para baixo. 
        # Para a seta ficar intuitiva na tela, usamos o ângulo visual direto.
        yaw_visual_rad = yaw_rad 
        arrow_len = 40
        end_x = int(cx + arrow_len * math.cos(yaw_visual_rad))
        end_y = int(cy + arrow_len * math.sin(yaw_visual_rad))
        cv.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 255), 2)

    return frame, dados_output

def main():
    # Setup com C920 Params automáticos
    cap, dynamic_params = setup_camera()
    detector = detector_apriltag()
    
    print("\n--- SENSOR GLOBAL ATIVO ---")
    print(f"Parâmetros em uso: fx={dynamic_params[0]:.1f}, fy={dynamic_params[1]:.1f}")
    print("Pressione 'q' para encerrar.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: 
                print("Falha no frame")
                break

            # Processamento (passando os params dinâmicos)
            frame_hud, dados = processar_frame_sensor(frame, detector, dynamic_params)

            # Exibe no terminal apenas se detectar algo (para não spammar)
            if dados:
                print(f"Dados: {dados}")

            cv.imshow("Sensor Global C920", frame_hud)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()