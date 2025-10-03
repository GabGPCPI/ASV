#! /usr/bin/env python3

from pupil_apriltags import Detector
import cv2
import numpy as np
from math import atan2
import socket
import time
import traceback

class main_class():

    def __init__(self, camera_index=1, show_window=True, rotate=0):
        """
        rotate: 0, 90, 180, 270
        90 = rotate clockwise 90° etc.
        """
        self.rotate = rotate
        self._refs_rotated = False
        self.initial_time = time.time()
        self.timenow = self.initial_time

        # alvo(s) — você pode alterar esses valores (são em pixels do sistema warp)
        self.target_list = [[220,340,0]]
        self.waypoint = 0
        self.psiLOS = 0
        self.truePsi = 0
        self.ePsi = 0
        self.main_image = False
        self.udp_ok = True
        self.show_img = show_window
        self.camera_index = camera_index

        if len(self.target_list) > 0 and len(self.target_list[self.waypoint]) >= 2:
            self.target = [int(self.target_list[self.waypoint][0]), int(self.target_list[self.waypoint][1])]
        else:
            self.target = [50, 50]

        # Dimensões reais X,Y em cm (se usa para escala)
        self.area = [220,146] # X,Y cm
        # Tamanho da janela de exibição (pixels)
        self.video_size = [440,292] # width, height
        # referências iniciais (quad markers) no frame original — serão rotacionadas se necessário
        self.references = [[0,0],[0,0],[0,0],[0,0]]#[[21,38],[21,451],[618,451],[618,38]] # tags 1,2,3,4

        # controle
        self.FpProp = 50
        self.FpsiProp = 0
        self.KPpsi = 40.0
        self.KvPpsi = 1.0
        self.KIpsi = 0.0
        self.integralErrorPsi = 0
        self.usv_psi_old = 0
        self.vPsi = 0
        self.newvPsi = 1
        self.angle = 0
        self.timenow5=time.time()

        # mapeamento warp -> display
        self._warp_wh = None  # (w,h) definidos quando faz warp (dst)

        # parâmetros de comando (ajuste)
        self.CMD_MAX = 999
        self.CMD_SCALE_FWD = 2.0  # pixels -> comando (ajuste)
        self.filename = time.strftime("%Y-%m-%d-%H-%M-%S")

    # ---- rotação utilitários ----
    def rotate_frame(self, frame):
        if self.rotate == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            return frame

    def rotate_point(self, pt, w, h):
        """Retorna a coordenada do ponto 'pt' após rotacionar a imagem original w x h para a orientação pedida."""
        x, y = int(pt[0]), int(pt[1])
        if self.rotate == 90:
            return [h - 1 - y, x]
        elif self.rotate == 180:
            return [w - 1 - x, h - 1 - y]
        elif self.rotate == 270:
            return [y, w - 1 - x]
        else:
            return [x, y]

    # ---- algoritmo de direção (normalizado) ----
    def target_alg(self, usv_x, usv_y):
        dx = (self.target[0] - usv_x)
        dy = (self.target[1] - usv_y)
        denom = np.hypot(dx, dy)
        if denom == 0 or denom < 1e-6:
            return [0.0, 0.0]
        g10 = dx / denom
        g20 = dy / denom
        return [g10, g20]

    def smallestAngle(self, currentAngle, targetAngle) -> float:
        """currentAngle e targetAngle em graus. retorna diferença mínima (target - current) em graus, sinalizado."""
        diff = (targetAngle - currentAngle) % 360.0
        if diff > 180.0:
            diff -= 360.0
        return diff

    def propulsion(self, usv_x, usv_y, usv_psi, g10, g20):
        """
        usv_x, usv_y : posição do robô no sistema (pixels do warp)
        usv_psi : orientação atual do robô (radianos, como você passa com atan2)
        g10,g20 : direção unitária para o target (vx,vy)
        Retorna [speedFWD, speedTURN] strings (formato zfilled).
        """
        psiLOS = atan2(g20, g10)
        # compute angular error: convert to degrees, compute smallest angle, then to radians
        ePsi_deg = self.smallestAngle(np.degrees(usv_psi), np.degrees(psiLOS))
        ePsi = np.radians(ePsi_deg)
        self.truePsi = usv_psi

        now = time.time()
        dt = now - self.timenow
        if dt <= 1e-6:
            dt = 1e-3

        self.integralErrorPsi += (ePsi_deg * dt)  # integrator in degrees
        self.psiLOS = psiLOS
        self.ePsi = ePsi

        # estimate yaw rate (deg/s) smoothing
        alphaa = 0.2
        deg_usv = np.degrees(usv_psi)
        # robust handling around +/- wrap
        if self.usv_psi_old > 90.0 and self.vPsi > 0 and deg_usv < 0.0:
            usv_psi2 = deg_usv + 360.0
            self.newvPsi = self.vPsi * (1.0 - alphaa) + alphaa * ((usv_psi2 - self.usv_psi_old) / dt)
            self.usv_psi_old = usv_psi2
        elif self.usv_psi_old < 90.0 and self.vPsi < 0 and deg_usv > 0.0:
            usv_psi2 = deg_usv - 360.0
            self.newvPsi = self.vPsi * (1.0 - alphaa) + alphaa * ((usv_psi2 - self.usv_psi_old) / dt)
            self.usv_psi_old = usv_psi2
        else:
            self.newvPsi = self.vPsi * (1.0 - alphaa) + alphaa * ((deg_usv - self.usv_psi_old) / dt)
            self.usv_psi_old = deg_usv

        if not np.isfinite(self.newvPsi):
            self.newvPsi = self.vPsi

        if abs(max([self.newvPsi, self.vPsi]) - min([self.newvPsi, self.vPsi])) > 10.0:
            self.newvPsi = self.vPsi
        else:
            self.vPsi = self.newvPsi

        self.vPsi = max(min(self.vPsi, 30.0), -30.0)
        vPsi_safe = self.vPsi if np.isfinite(self.vPsi) else 0.0

        # PI/PID yaw controller (simplified)
        FpsiProp = (self.KPpsi * ePsi) - (self.KvPpsi * vPsi_safe) + (self.KIpsi * self.integralErrorPsi)
        if not np.isfinite(FpsiProp):
            FpsiProp = 0.0
        self.FpsiProp = float(FpsiProp)

        # saturate & bias (mantive sua formula)
        if self.FpsiProp > 0:
            self.FpsiProp = 0.7 * abs(self.FpsiProp) + 400.0
        elif self.FpsiProp < 0:
            self.FpsiProp = -(0.7 * abs(self.FpsiProp) + 400.0)

        # forward force proportional à distância
        dist_pix = np.hypot(usv_x - self.target[0], usv_y - self.target[1])
        fp = int(min(max(dist_pix * self.CMD_SCALE_FWD, 0), self.CMD_MAX))
        #self.FpProp = fp

        # waypoint switching
        '''reach_threshold = 5  # pixels
        if dist_pix <= reach_threshold:
            if self.waypoint < len(self.target_list) - 1:
                self.waypoint += 1
                self.target = [int(self.target_list[self.waypoint][0]), int(self.target_list[self.waypoint][1])]
                print(f"[WAYPOINT] avançou para {self.waypoint}, novo target {self.target}")
            else:
                self.FpProp = 0
                print("[WAYPOINT] último alvo alcançado. Parando forward.")'''

        # formata comandos (veja nota abaixo sobre sinal/convenção)
        mag_turn = int(min(max(int(abs(self.FpsiProp*2.5)), 0), self.CMD_MAX))
        # OBS: aqui eu mantenho a sua convenção original: '-' quando FpsiProp >=0 e '+' quando FpsiProp <0.
        # Se sua plataforma espera o contrário, troque a lógica do sinal aqui.
        sign = '-' if self.FpsiProp >= 0 else '+'
        speedTURN = sign + str(mag_turn).zfill(4)

        mag_fwd = int(min(max(int(abs(self.FpProp*4.5)), 0), self.CMD_MAX))
        speedFWD = str(mag_fwd).zfill(4)

        self.timenow = now
        return [speedFWD, speedTURN]

    def image_processor(self):
        # prefer backends
        preferred_backends = [
            (cv2.CAP_DSHOW, "CAP_DSHOW"),
            (cv2.CAP_MSMF,  "CAP_MSMF"),
            (cv2.CAP_VFW,   "CAP_VFW"),
        ]

        cap = None
        used_backend_name = None
        for backend, name in preferred_backends:
            cap = cv2.VideoCapture(self.camera_index, backend)
            if cap is not None and cap.isOpened():
                print(f"[OK] Opened camera index={self.camera_index} with backend={name}")
                used_backend_name = name
                break
            else:
                print(f"[FAIL] Could not open index={self.camera_index} with backend={name}")
                try:
                    if cap is not None:
                        cap.release()
                except:
                    pass
                cap = None

        if cap is None or not cap.isOpened():
            print("Erro: não foi possível abrir a câmera com os backends testados.")
            return

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

        # Detector (pupil_apriltags)
        try:
            detector = Detector(
                families="tag36h11",
                nthreads=2,
                quad_decimate=2.0,
                quad_sigma=0.0,
                refine_edges=0,
                decode_sharpening=0.0,
                debug=0
            )
            use_pupil = True
        except Exception:
            import apriltag
            options = apriltag.DetectorOptions(families='tag36h11')
            detector = apriltag.Detector(options)
            use_pupil = False

        # independent timing para full-frame e warp-frame
        detection_interval_full = 0.20
        detection_interval_warp = 0.20
        last_detect_time_full = 0.0
        last_detect_time_warp = 0.0

        print(f"Starting main loop (camera index={self.camera_index}, backend={used_backend_name}, rotate={self.rotate})")
        # ajuste o HOST e PORT para o IP do seu robo
        PORT = 2222
        # HOST = '192.168.0.80'  #Robo 1
        HOST = "192.168.0.83" #Robo 2
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = (HOST, PORT)

        try:
            while True:
                ret, frame = cap.read()
                if not hasattr(self, '_printed_frame_info'):
                    print("DEBUG frame read:", ret, "shape:", None if frame is None else getattr(frame, "shape", None))
                    self._printed_frame_info = True
                if not ret or frame is None:
                    time.sleep(0.05)
                    continue

                # rotaciona referências somente 1x, baseado nas dimensões originais
                '''if not self._refs_rotated and self.rotate in (90,180,270):
                    h0, w0 = frame.shape[:2]  # dims originais
                    new_refs = []
                    for rp in self.references:
                        new_refs.append(self.rotate_point(rp, w0, h0))
                    self.references = new_refs
                    self._refs_rotated = True
                    print("[INFO] references rotacionadas para a nova orientação.")'''

                # agora rotaciona frame conforme pedido
                frame = self.rotate_frame(frame)

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                #now = time.time()
                results_full = []
                '''if now - last_detect_time_full >= detection_interval_full:
                    try:
                        t0 = time.time()'''
                results_full = detector.detect(gray)
                '''        dt = time.time() - t0
                        if dt > 0.5:
                            print(f"Detector full-frame lento: {dt:.2f}s")
                    except Exception as e:
                        print("Detector full-frame error:", e)
                        traceback.print_exc()
                        results_full = []
                    last_detect_time_full = now'''

                # tenta perspectiva (warp) se referências válidas
                '''dst = None
                colored = None
                results_warp = []
                self._warp_wh = None

                if all(isinstance(ref, (list,tuple)) and len(ref)==2 for ref in self.references):
                    try:
                        pts1 = np.float32(self.references)
                        warp_scale = 5
                        w = int(self.area[0] * warp_scale)
                        h = int(self.area[1] * warp_scale)
                        pts2 = np.float32([[0, 0],
                                           [0, h],
                                           [w, h],
                                           [w, 0]])
                        M = cv2.getPerspectiveTransform(pts1, pts2)
                        dst = gray #cv2.warpPerspective(gray, M, (w, h))
                        colored = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
                        #self._warp_wh = (w, h)
                        # detect on warp with independent interval
                        if now - last_detect_time_warp >= detection_interval_warp:
                            try:
                                t0 = time.time()
                                results_warp = detector.detect(dst)
                                dt = time.time() - t0
                                if dt > 0.5:
                                    print(f"Detector warp-frame lento: {dt:.2f}s")
                            except Exception as e:
                                print("Detector warp-frame error:", e)
                                traceback.print_exc()
                                results_warp = []
                            last_detect_time_warp = now
                    except Exception as e:
                        print("Erro no perspective transform / detect:", e)
                        traceback.print_exc()
                        dst = None
                        colored = None
                        results_warp = []
                        self._warp_wh = None

                # ===== PROCESSA DETECÇÕES NO WARP (preferido para posições mais precisas) =====
                if results_warp:
                    for r in results_warp:
                        try:
                            (ptA, ptB, ptC, ptD) = r.corners
                            ptB = (int(ptB[0]), int(ptB[1]))
                            ptC = (int(ptC[0]), int(ptC[1]))
                            ptD = (int(ptD[0]), int(ptD[1]))
                            ptA = (int(ptA[0]), int(ptA[1]))
                            (cX, cY) = (int(r.center[0]), int(r.center[1]))

                            # desenha target (no sistema warp)
                            cv2.circle(colored, (self.target[0], self.target[1]), 60, (0, 255, 255), 2) # TARGET warp

                            # eixo do tag
                            cv2.line(colored, (cX, cY), (int((ptB[0]+ptC[0])/2), int((ptB[1]+ptC[1])/2)), (0, 0, 255), 2)
                            cv2.line(colored, (cX, cY), (int((ptA[0]+ptB[0])/2), int((ptA[1]+ptB[1])/2)), (0, 255, 0), 2)
                            cv2.circle(colored, (cX, cY), 5, (255, 0, 0), -1)

                            # desenha linhas entre waypoints (corrigido índices)
                            for ii in range(len(self.target_list)-1):
                                cv2.line(colored,
                                         (self.target_list[ii][0], self.target_list[ii][1]),
                                         (self.target_list[ii+1][0], self.target_list[ii+1][1]),
                                         (255, 255, 255), 2)

                            angle = atan2(int((ptB[1]+ptC[1])/2)-cY, int((ptB[0]+ptC[0])/2)-cX)
                            coord = {'id':r.tag_id, 'x':cX, 'y':cY, 'psi':np.degrees(angle)}

                            # somente tags diferentes das referências (1..4) são consideradas como "robo"
                            if r.tag_id not in [1,2,3,4]:
                                tgtR = self.target_alg(cX, cY)
                                speedR = self.propulsion(cX, cY, angle, tgtR[0], tgtR[1])

                                info_usv = ["x: "+str(int(cX)),
                                            "y: "+str(int(cY)),
                                            "yaw: "+str(int(np.degrees(angle))),
                                            "target yaw: "+str(int(np.degrees(self.psiLOS))),
                                            "ePsi: "+str(int(np.degrees(self.ePsi))),
                                            "yaw_rate: "+str(int(self.vPsi)),
                                            "force yaw: "+speedR[1],
                                            "time: "+str(time.time())]
                                cnt_txt = 20
                                for txt in info_usv:
                                    cv2.putText(colored, txt, (10, cnt_txt), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1,2)
                                    cnt_txt+=15

                                # monta mensagem e envia via UDP
                                msg3 = "{"+speedR[1]+speedR[0]+"}"
                                print(f"[WARP] tag_id={r.tag_id} at ({cX},{cY}) angle={np.degrees(angle):.1f}deg -> msg SEND: {msg3} -> {dest}")
                                if self.udp_ok:
                                    try:
                                        udp.sendto(msg3.encode(), dest)
                                    except Exception as e:
                                        print("UDP send error:", e)
                                        self.udp_ok = False
                            else:
                                # se for referência, só desenha e atualiza self.references (caso esteja em full-frame as atualização é no bloco abaixo)
                                cv2.putText(colored, f"ref:{r.tag_id}", (cX+5, cY+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,0), 1,2)

                        except Exception:
                            traceback.print_exc()
                            continue'''

                # ===== PROCESSA DETECÇÕES NO FULL FRAME (fallback) =====
                gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                if results_full:
                    for r in results_full:
                        try:
                            (ptA, ptB, ptC, ptD) = r.corners
                            ptB = (int(ptB[0]), int(ptB[1]))
                            ptC = (int(ptC[0]), int(ptC[1]))
                            ptD = (int(ptD[0]), int(ptD[1]))
                            ptA = (int(ptA[0]), int(ptA[1]))
                            (cX, cY) = (int(r.center[0]), int(r.center[1]))

                            # atualiza refs se for 1..4
                            #if r.tag_id==1: self.references[0]=[cX,cY]
                            #if r.tag_id==7: self.references[1]=[cX,cY]
                            #if r.tag_id==3: self.references[2]=[cX,cY]
                            #if r.tag_id==4: self.references[3]=[cX,cY]
                            #print()

                            # se não for referencia, considera robo e comanda
                            if 1==1:#r.tag_id not in [1,7,3,4]:
                                cv2.line(gray_bgr, (cX, cY), (int((ptB[0]+ptC[0])/2), int((ptB[1]+ptC[1])/2)), (0, 0, 255), 2)
                                cv2.line(gray_bgr, (cX, cY), (int((ptA[0]+ptB[0])/2), int((ptA[1]+ptB[1])/2)), (0, 255, 0), 2)
                                cv2.circle(gray_bgr, (cX, cY), 5, (255, 0, 0), -1)

                                tx = int(self.target[0])
                                ty = int(self.target[1])
                                cv2.circle(gray_bgr, (tx, ty), 8, (0,255,255), 2)
                                cv2.putText(gray_bgr, f"Target:{self.target}", (tx+10, ty+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1,2)

                                angle = atan2(int((ptB[1]+ptC[1])/2)-cY, int((ptB[0]+ptC[0])/2)-cX)
                                tgtR = self.target_alg(cX, cY)
                                speedR = self.propulsion(cX, cY, angle, tgtR[0], tgtR[1])

                                info_usv = ["x: "+str(int(cX)),
                                            "y: "+str(int(cY)),
                                            "yaw: "+str(int(np.degrees(angle))),
                                            "yaw_rate: "+str(int(self.vPsi)),
                                            "force yaw: "+speedR[1],
                                            "time: "+str(time.time())]
                                cnt_txt = 50
                                for txt in info_usv:
                                    cv2.putText(gray_bgr, txt, (10, cnt_txt), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1,2)
                                    cnt_txt+=15

                                msg3 = "{"+speedR[1]+speedR[0]+"}"
                                print(f"[FULL] tag_id={r.tag_id} at ({cX},{cY}) angle={np.degrees(angle):.1f}deg -> msg SEND: {msg3} -> {dest}")
                                if self.udp_ok:
                                    try:
                                        udp.sendto(msg3.encode(), dest)
                                    except Exception as e:
                                        print("UDP send error:", e)
                                        self.udp_ok = False

                        except Exception:
                            traceback.print_exc()
                            continue

                # ==== DISPLAY ====
                '''display_frame = None
                if self.rotate in (90, 270):
                    vw, vh = (self.video_size[1], self.video_size[0])
                else:
                    vw, vh = (self.video_size[0], self.video_size[1])

                if colored is not None:
                    try:
                        display_frame = cv2.resize(colored, (vw, vh))
                    except Exception as e:
                        print("Warning: resize transformed image failed:", e)
                        display_frame = None

                if display_frame is None and frame is not None:
                    try:
                        display_frame = cv2.resize(gray_bgr, (vw, vh))
                    except Exception as e:
                        print("Warning: resize original frame failed:", e)
                        display_frame = gray_bgr

                if display_frame is not None:
                    if self._warp_wh is not None:
                        warp_w, warp_h = self._warp_wh
                        sx = vw / float(warp_w) if warp_w>0 else 1.0
                        sy = vh / float(warp_h) if warp_h>0 else 1.0
                        tx = int(self.target[0] * sx)
                        ty = int(self.target[1] * sy)
                        if 0 <= tx < vw and 0 <= ty < vh:
                            cv2.circle(display_frame, (tx, ty), 8, (0,255,255), 2)
                            cv2.putText(display_frame, f"Target:{self.target}", (tx+10, ty+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1,2)
                    else:
                        cv2.putText(display_frame, f"Target:{self.target}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2,2)'''

                if self.show_img:
                    #gray_bgr = cv2.rotate(gray_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    cv2.imshow('frame', gray_bgr)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Saindo por tecla 'q'")
                    break

        except KeyboardInterrupt:
            print("Interrompido pelo usuário (Ctrl+C).")
        except Exception as e:
            print("Erro inesperado:")
            traceback.print_exc()
        finally:
            try:
                cap.release()
            except:
                pass
            cv2.destroyAllWindows()
            print("Camera and windows released/closed.")


if __name__ == '__main__':
    # rotate=90 para girar 90° clockwise se seu dispositivo precisar
    Image = main_class(camera_index=1, show_window=True, rotate=90)
    Image.image_processor()