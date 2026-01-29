from pupil_apriltags import Detector
import numpy as np
import cv2 as cv
import time
import os
import math

# --- CONSTANTES DE VISÃO (SENSOR) ---
# Tamanho real da lateral preta da AprilTag (EM METROS). 
TAG_SIZE = 0.07439341

# Parâmetros da Câmera (fx, fy, cx, cy).
# Sem calibração, usamos valores aproximados para webcam padrão.
# Se a imagem for 640x480, o centro é aprox 320, 240.
# O foco (fx, fy) aproximado é geralmente a largura da imagem (ex: 600 a 800).
CAMERA_PARAMS = [600, 600, 320, 240]

def camera_config():
    """
    Mantida do original.
    """
    print("--- Configuração da Câmera ---")
    print("Escolha uma predefinição:")
    print("  [1]: 640x480 @ 30 FPS (Padrão)")
    print("  [2]: 1280x720 (HD)")
    
    escolha = input("Digite sua escolha (1 ou 2): ").strip()
    if escolha == "2":
        return 1280, 720, 30.0
    else:
        return 640, 480, 30.0

def setup_camera():
    """
    Mantida do original (simplificada).
    """
    width, height, fps_req = camera_config()
    cap = cv.VideoCapture(0, cv.CAP_DSHOW) # Tente indice 0 ou 1
    
    if not cap.isOpened():
        print("Erro ao abrir câmera.")
        exit()
        
    cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv.CAP_PROP_FPS, fps_req)
    
    print(f"Câmera iniciada: {width}x{height}")
    return cap

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

def processar_frame_sensor(frame, detector):
    """
    VERSÃO LIMPA: Apenas detecta e extrai dados métricos.
    Não calcula motores, não tem PID, não tem Waypoints.
    """
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # AQUI ESTÁ A MUDANÇA CRÍTICA: estimate_tag_pose=True
    # Isso exige camera_params e tag_size
    tags = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE
    )

    dados_output = {}

    for tag in tags:
        # Extração de Pose 3D (Matemática de TCC)
        pose_t = tag.pose_t # Vetor Translação [x, y, z] em metros
        pose_R = tag.pose_R # Matriz de Rotação 3x3

        # Cálculo do Yaw (Rotação no eixo Z) a partir da matriz
        yaw_rad = math.atan2(pose_R[1, 0], pose_R[0, 0])
        yaw_deg = -math.degrees(yaw_rad) #Negativo somente para passar do padrão trig para o NED do Fossen

        # Dados estruturados
        # Assumindo câmera Top-Down (olhando para baixo):
        # X da imagem = X do Mundo
        # Y da imagem = Y do Mundo (Cuidado com o sinal do Y em visão computacional!)
        id_robo = tag.tag_id
        x_m = pose_t[0][0]
        y_m = pose_t[1][0]
        
        dados_output[id_robo] = {'x': x_m, 'y': y_m, 'yaw': yaw_deg}

        # --- DESENHO NA TELA (VISUALIZAÇÃO APENAS) ---
        cx, cy = int(tag.center[0]), int(tag.center[1])
        
        # 1. Centro e ID
        cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv.putText(frame, f"ID:{id_robo}", (cx+10, cy-10), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 2. Dados Métricos (HUD Sensor)
        cv.putText(frame, f"X:{x_m:.2f}m Y:{y_m:.2f}m", (cx+10, cy+10), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv.putText(frame, f"Yaw:{yaw_deg:.1f}", (cx+10, cy+25), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # 3. Seta de Orientação (Mostra para onde o robô "acha" que é frente)
        comprimento_seta = 40
        end_x = int(cx + comprimento_seta * math.cos(yaw_rad))
        end_y = int(cy + comprimento_seta * math.sin(yaw_rad))
        cv.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 255), 2)

    return frame, dados_output

def main():
    # Setup Inicial
    cap = setup_camera()
    detector = detector_apriltag()
    
    print("\n--- SENSOR GLOBAL ATIVO ---")
    print("Mostrando apenas dados. Sem controle.")
    print("Pressione 'q' para sair.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Processamento
        frame_hud, dados = processar_frame_sensor(frame, detector)

        # Exibe dados no terminal (para você validar)
        if dados:
            # Limpa terminal ou imprime linha a linha
            print(f"Leitura: {dados}")

        # Mostra Vídeo
        cv.imshow("Sensor Global", frame_hud)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()