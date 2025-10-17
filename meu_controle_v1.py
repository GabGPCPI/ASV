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
fourcc = cv.VideoWriter_fourcc(*'XVID')

#Exemplo: "2025-10-17_16-55-00" (Ano-Mês-Dia_Hora-Minuto-Segundo)
timestamp = time.strftime("%Y-%m-%d_%H-%M-%S") #Obter a string de data/hora atual formatada

pasta_destino = 'C:/Users/Gabriel/Desktop/PySwarming/Video salvos/'

nome_arquivo = f"{pasta_destino}gravacao_{timestamp}.avi"
print(f"Salvando vídeo em: {nome_arquivo}") # Informa ao usuário onde o arquivo será salvo

grava_e_salva_video = cv.VideoWriter(nome_arquivo, fourcc, 30.0, (640,  480))

while captura_de_video.isOpened():

    reticulado, frame = captura_de_video.read() #Lê um frame da câmera. 'reticulado' será True se a leitura foi bem-sucedida.

    if not reticulado: #sem leitura de imagem
        print("Houve um erro ao capturar o frame, encerrando o programa.")
        break

    cinza = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #define a cor da janela - tons de cinza

    #frame = cv.flip(frame, 0) #deixa o video de ponta cabeça
    #grava_e_salva_video.write(cinza) #salva o video em tons de cinza
    grava_e_salva_video.write(frame) #salva o video colorido

    cv.imshow("Minha câmera", cinza) #Mostra o frame na janela na cor cinza

    if cv.waitKey(1) & 0xFF == ord('e'): 
        #Espera 1ms por uma tecla. Se a tecla 'e' for pressionada, encerra o loop.
        print("Tecla 'e' - a saída do programa foi ativada")
        break

print("Liberando a câmera, salvando o video e fechando as janelas de video")
captura_de_video.release()
grava_e_salva_video.release()
cv.destroyAllWindows()