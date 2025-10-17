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

        #__init__ em Python é um método especial chamado construtor que inicializa um objeto da classe. Sua principal tarefa é preparar o objeto, definindo todas as suas variáveis iniciais (chamadas de atributos) para que ele esteja pronto para trabalhar.'
        
        #camera_index=0, show_window=True, rotate=0: Estes são os parâmetros que você pode passar ao criar o objeto. Eles já vêm com valores padrão:'
        #camera_index: Qual câmera usar (0 é a padrão/integrada).'
        #show_window: Se True, uma janela aparecerá mostrando a imagem da câmera.'
        #rotate: Permite girar a imagem da câmera caso ela esteja de lado.'
        
        """
        rotate: 0, 90, 180, 270
        90 = rotate clockwise 90° etc.
        """
        self.rotate = rotate #self.rotate = rotate: Guarda o valor do ângulo de rotação em uma variável interna do objeto.
        self._refs_rotated = False #self._refs_rotated = False: Esta é uma variável de controle (um "flag"). Ela começa como False para garantir que uma operação de rotação de pontos de referência só aconteça uma única vez.
        self.initial_time = time.time() #self.initial_time = time.time(): Pega o tempo exato (em segundos) em que o programa começou e o armazena.
        self.timenow = self.initial_time #'self.timenow = self.initial_time: timenow será usada para calcular intervalos de tempo (dt) no loop de controle. Ela começa com o mesmo valor do tempo inicial.'

        # Definição do Alvo (Waypoint) e Variáveis de Navegação
        # alvo(s) — você pode alterar esses valores (são em pixels do sistema warp)
        self.target_list = [[220,340,0]] #Uma lista que contém as coordenadas de todos os alvos que o ASV deve seguir em sequência.
        self.waypoint = 0 #O índice do alvo atual na target_list
        #self.psiLOS, self.truePsi, self.ePsi: Estas são variáveis cruciais para a navegação
        self.psiLOS = 0 #(Psi Line-of-Sight) Guardará o ângulo desejado, ou seja, a direção que o ASV deveria apontar para ir em direção ao alvo.
        self.truePsi = 0 #Guardará o ângulo real do ASV, medido pela AprilTag.
        self.ePsi = 0 #(Erro de Psi) Guardará a diferença entre o ângulo desejado e o ângulo real. É esse erro que o controlador tentará zerar.
        self.main_image = False
        self.udp_ok = True
        self.show_img = show_window
        self.camera_index = camera_index

        if len(self.target_list) > 0 and len(self.target_list[self.waypoint]) >= 2:
            self.target = [int(self.target_list[self.waypoint][0]), int(self.target_list[self.waypoint][1])]
        else:
            self.target = [50, 50]

        #O bloco de código acima simplesmente pega o primeiro waypoint da target_list e o define como o self.target ativo. Se a lista estiver vazia, ele define um alvo padrão [50, 50].
        #O objetivo do bloco de código é pegar o alvo atual da target_list e colocá-lo em uma variável mais simples, a self.target, que será usada nos cálculos de navegação.
        'len(self.target_list) > 0'
            #'O que faz? len() calcula o número de itens em uma lista. Esta parte verifica se a target_list não está vazia.'
            #'Por que é importante? Se a lista de alvos estivesse vazia e o programa tentasse acessar um item dela, ocorreria um erro (IndexError) e o script travaria. Esta é a primeira barreira de segurança: Existe pelo menos um alvo para seguir?.'

        'len(self.target_list[self.waypoint]) >= 2'
            #'O que faz? Esta parte só é verificada se a Parte 1 for verdadeira.'
            #'self.target_list[self.waypoint]: Acessa o alvo atual dentro da lista. Com self.waypoint = 0, isso resulta em [220, 340, 0].'
            #'len(...) >= 2: Verifica se esse alvo tem pelo menos dois elementos, ou seja, uma coordenada X e uma coordenada Y.'
            #'Por que é importante? Garante que o alvo definido na lista está formatado corretamente. Se um programador definisse acidentalmente um alvo como [100] (apenas um número), o código de navegação que espera target[0] (X) and target[1] (Y) falharia. Esta é a segunda barreira de segurança: "O alvo que encontramos tem, no mínimo, as coordenadas X e Y?".'

        'O que acontece dentro do if'
        'Se a condição for VERDADEIRA:'
        'self.target = [int(self.target_list[self.waypoint][0]), int(self.target_list[self.waypoint][1])'
            #'self.target_list[self.waypoint][0]: Pega o primeiro elemento do alvo atual (o valor 220).'
            #'self.target_list[self.waypoint][1]: Pega o segundo elemento do alvo atual (o valor 340).'
            #'int(...): Garante que os valores sejam números inteiros'
            #'Resultado: A variável self.target é criada e se torna [220, 340]. O programa agora tem um alvo claro para o qual navegar.'

        # Dimensões reais X,Y em cm (se usa para escala)
        self.area = [220,146] # X,Y cm
        # Tamanho da janela de exibição (pixels)
        'self.area: Dimensões do tanque ou área de teste em centímetros. Seria usado para converter pixels em medidas reais, mas essa funcionalidade (chamada de "warp perspective") está comentada no código.'
        self.video_size = [440,292] # width, height
        # referências iniciais (quad markers) no frame original — serão rotacionadas se necessário
        'self.video_size: O tamanho em pixels que a janela de exibição terá. A imagem da câmera será redimensionada para este tamanho.'
        self.references = [[0,0],[0,0],[0,0],[0,0]]#[[21,38],[21,451],[618,451],[618,38]] # tags 1,2,3,4
        'A lista acima guarda as posições (em pixels) de 4 AprilTags fixas nos cantos da área de teste. A ideia era usar essas tags de referência para corrigir a perspectiva da câmera e obter uma visão "de cima" perfeitamente retangular. No entanto, como a seção de código que usa isso está desativada, essas referências não têm efeito no momento.'

        # controle
        self.FpProp = 50 #Guardará os valores calculados de força de propulsão para frente
        self.FpsiProp = 0 #Guardará os valores calculados de força de giro
        self.KPpsi = 40.0 #(Ganho Proporcional): O mais importante. Multiplica o erro de ângulo (ePsi). Um KP alto significa que o ASV reagirá de forma agressiva a qualquer desvio, virando bruscamente. Um KP baixo o torna mais lento e suave na correção.
        self.KvPpsi = 1.0 #(Ganho Derivativo): Multiplica a velocidade com que o ângulo está mudando (a "taxa de guinada" ou yaw rate). Funciona como um amortecedor. Se o ASV está virando muito rápido, esse termo atua no sentido contrário para frear a rotação, evitando que ele "passe do ponto" e comece a oscilar.
        self.KIpsi = 0.0 #(Ganho Integral): Está definido como 0.0, então o controlador é, na verdade, um PD (Proporcional-Derivativo). Se houvesse um erro pequeno e constante (por exemplo, devido a uma correnteza leve), o termo I acumularia esse erro ao longo do tempo até gerar uma força de correção suficiente para vencê-lo.
        self.integralErrorPsi = 0 #É o Acumulador do Erro Integral. Ele soma os erros de orientação ao longo do tempo.
        self.usv_psi_old = 0 #usv_psi_old guarda o ângulo do ciclo anterior para poder calcular a velocidade de rotação (vPsi).
        self.vPsi = 0 #self.vPsi: Guarda o valor suavizado ou filtrado da velocidade angular, que será efetivamente usado no cálculo do controle.
        self.newvPsi = 1 #self.newvPsi: Guarda o valor "bruto" da velocidade angular, recém-calculado.
        self.angle = 0
        self.timenow5=time.time()


        # mapeamento warp -> display
        self._warp_wh = None  # (w,h) definidos quando faz warp (dst)

        # parâmetros de comando (ajuste)
        self.CMD_MAX = 999 #self.CMD_MAX: Define o valor numérico máximo que os comandos de motor podem ter. O valor 999 foi escolhido porque o comando final é formatado com 4 dígitos (ex: 0999).
        self.CMD_SCALE_FWD = 2.0  #self.CMD_SCALE_FWD: É um fator de multiplicação. A força de avanço é calculada como distancia_em_pixels * CMD_SCALE_FWD. Se você aumentar esse valor, o ASV andará mais rápido quando estiver longe do alvo.
        self.filename = time.strftime("%Y-%m-%d-%H-%M-%S") # Cria um nome de arquivo único baseado na data e hora atuais, útil para salvar logs ou imagens se essa funcionalidade fosse adicionada.

    # ---- rotação utilitários ----
    def rotate_frame(self, frame):
        #Sua única responsabilidade é pegar um frame (a imagem inteira capturada pela câmera) e girá-lo fisicamente.
        'Entrada: Ela recebe self (acesso às variáveis do objeto, como self.rotate) e frame (que é a imagem em si, representada como uma matriz de pixels pela biblioteca OpenCV).'
        'Lógica: Ela usa uma série de if/elif/else para verificar o valor da variável self.rotate, que foi definida quando o objeto main_class foi criado.'
        if self.rotate == 90:
            'if self.rotate == 90:: Se o valor for 90, ela usa a função cv2.rotate() da biblioteca OpenCV com a constante cv2.ROTATE_90_CLOCKWISE para girar a imagem 90 graus no sentido horário.'
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == 180:
            'elif self.rotate == 180:: Se for 180, ela gira a imagem de ponta-cabeça.'
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate == 270:
            'Se for 270, ela gira 90 graus no sentido anti-horário (COUNTERCLOCKWISE).'
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            'else:: Se self.rotate for 0 ou qualquer outro valor, ela não faz nada e simplesmente retorna o frame original.'
            return frame

    def rotate_point(self, pt, w, h):
        #Essa função lida com a matemática da rotação, mas para um ponto individual em vez da imagem toda. Ela é necessária porque, se você gira a imagem, as coordenadas de qualquer ponto específico (como os cantos de uma AprilTag ou um alvo) também mudam de lugar."""
        'pt: Um ponto com as coordenadas originais, por exemplo [x, y].'
        'w: A largura (width) da imagem antes da rotação.'
        'h: A altura (height) da imagem antes da rotação.'
        x, y = int(pt[0]), int(pt[1])
        if self.rotate == 90:
            'Nova coordenada X: A nova posição horizontal se torna h - 1 - y. Pense nisso: o que era a coordenada vertical y em uma imagem em pé, se torna a coordenada horizontal em uma imagem deitada. O h - 1 é usado para inverter a direção (pois o eixo Y começa de cima para baixo em imagens).'
            'Nova coordenada Y: A nova posição vertical se torna a coordenada x original. O que era horizontal agora é vertical.'
            return [h - 1 - y, x]
        elif self.rotate == 180:
            return [w - 1 - x, h - 1 - y]
        elif self.rotate == 270:
            return [y, w - 1 - x]
        else:
            return [x, y]

    'Nota: a parte do código que de fato usa a função rotate_point está comentada. Ela seria usada para rotacionar as coordenadas das AprilTags de referência (self.references) no início do programa. No estado atual, apenas rotate_frame tem um efeito prático no fluxo do código.'

    # ---- algoritmo de direção (normalizado) (Matemática da navegação) (Lógica de controle) ----
    def target_alg(self, usv_x, usv_y):
        #O nome target_alg significa "algoritmo do alvo". 
        #A única função dele é calcular um vetor de direção unitário que aponta do ASV para o alvo.
        'usv_x, usv_y: Posição atual do ASV.'
        'self.target[0], self.target[1]: Posição do alvo.'
        # 1. Calcula a diferença nas coordenadas X e Y
        dx = (self.target[0] - usv_x)
        dy = (self.target[1] - usv_y)
        'dx, dy: Componentes do vetor que vai do ASV até o alvo. Se dx for positivo, o alvo está à direita; se dy for positivo, o alvo está abaixo'
        # 2. Calcula a distância real (a hipotenusa)
        denom = np.hypot(dx, dy)
        'np.hypot(dx, dy) é uma função da biblioteca NumPy que calcula a hipotenusa de um triângulo retângulo com catetos dx e dy'
        'O resultado, denom, é a distância em linha reta (em pixels) entre o ASV e o alvo.'
        # 3. Medida de segurança
        if denom == 0 or denom < 1e-6:
            return [0.0, 0.0]
        'Este if previne uma divisão por zero. Se a distância for zero (ou muito, muito próxima de zero), significa que o ASV já chegou ao alvo. Nesse caso, ele retorna um vetor [0.0, 0.0], indicando que não há direção para se mover.'
        # 4. Normalização do vetor
        g10 = dx / denom
        g20 = dy / denom
        'Ao dividir cada componente do vetor (dx, dy) pela sua magnitude total (denom), o novo vetor [g10, g20] terá sempre um comprimento (magnitude) de 1.'
        return [g10, g20]

    def smallestAngle(self, currentAngle, targetAngle) -> float:
        #Esta função é uma ferramenta matemática muito inteligente para resolver um problema comum com ângulos: a "volta" no círculo de 360 graus.
        #O objetivo é simples: se você está virado para 10° e quer virar para 350°, qual é o caminho mais curto? Virar +340° para a direita ou virar -20° para a esquerda? Obviamente, virar -20° é mais rápido. Esta função calcula exatamente isso.
        ## 1. Calcula a diferença "bruta"
        diff = (targetAngle - currentAngle) % 360.0
        'targetAngle - currentAngle: Calcula a diferença simples. Usando nosso exemplo, 350 - 10 = 340.'
        '% 360.0: O operador % (módulo) calcula o resto de uma divisão. Isso garante que o resultado esteja sempre no intervalo de 0 a 359.9. No nosso exemplo, 340 % 360 ainda é 340. Se o resultado fosse, por exemplo, 400, 400 % 360 seria 40. Isso normaliza o ângulo para uma volta completa.'
        # 2. A "mágica" para encontrar o caminho mais curto
        if diff > 180.0:
            diff -= 360.0
            'Esta é a lógica principal. Se a diferença calculada for maior que 180 graus, significa que o caminho "por trás" é mais curto.'
            'No exemplo, diff é 340, que é maior que 180.'
            'Então, o código faz diff -= 360.0, que é 340 - 360 = -20.'
            'O resultado -20 é o menor ângulo. O sinal negativo indica que a rotação deve ser no sentido anti-horário (ou para a esquerda, dependendo da convenção).'                   
            'resultado sempre no intervalo (-180,+180)'
        return diff

    def propulsion(self, usv_x, usv_y, usv_psi, g10, g20):
        #Esta é, sem dúvida, a função mais importante do script Python. É o verdadeiro cérebro do ASV
        #A posição e a orientação são transformadas em comandos de motor concretos.
        """
        usv_x, usv_y : posição do robô no sistema (pixels do warp)
        usv_psi : orientação atual do robô (radianos, como você passa com atan2)
        g10,g20 : direção unitária para o target (vx,vy)
        Retorna [speedFWD, speedTURN] strings (formato zfilled).
        """
        #1. Cálculo do Erro de Orientação (Erro Proporcional - P)
        psiLOS = atan2(g20, g10)
        #psiLOS = atan2(g20, g10): Aqui, usamos o vetor de direção unitário [g10, g20] (calculado pela target_alg) para obter o ângulo desejado (psiLOS).
        #atan2 converte as componentes X e Y do vetor em um ângulo em radianos. Este é o ângulo da "Linha de Visada" (Line-of-Sight) para o alvo.
        ePsi_deg = self.smallestAngle(np.degrees(usv_psi), np.degrees(psiLOS)) 
        #Converte o ângulo atual do ASV (usv_psi) e o ângulo desejado (psiLOS) de radianos para graus.
        #Usa a função smallestAngle para calcular a diferença angular mais curta entre os dois.
        #O resultado, ePsi_deg, é o erro de orientação em graus. Se for positivo, o ASV precisa virar em um sentido; se for negativo, no outro.
        ePsi = np.radians(ePsi_deg)
        #ePsi = np.radians(ePsi_deg): O erro é convertido de volta para radianos (ePsi), que é a unidade usada nos cálculos do controlador.
        self.truePsi = usv_psi 
        #pega o valor do ângulo atual e o salva dentro do objeto.
        'Ao armazenar o ângulo em self.truePsi, o valor fica acessível fora da função propulsion. Se você quisesse, por exemplo, exibir o ângulo do robô na tela em tempo real ou registrá-lo em um arquivo de log, você usaria self.truePsi. Ele serve como um registro público do último ângulo medido.'

        #2. Medir o Tempo e Calcular o Erro Integral (O "I")
        
        now = time.time() #now = time.time(): Captura o tempo atual exato em segundos
        dt = now - self.timenow #Calcula o tempo decorrido (dt, de "delta time") desde a última vez que esta função foi executada.
        if dt <= 1e-6: #Uma proteção para evitar divisão por zero se o loop rodar rápido demais. Garante que dt seja sempre um número positivo pequeno.
            dt = 1e-3

        self.integralErrorPsi += (ePsi_deg * dt)  #Esta linha implementa o termo Integral (I) (Acumulador do Erro Integral. Ele soma os erros de orientação ao longo do tempo)
        #Ela pega o erro atual em graus (ePsi_deg), multiplica pelo intervalo de tempo (dt) e o soma ao valor acumulado em self.integralErrorPsi
        #(Lembre-se: como KIpsi é 0, este termo está desativado na prática).
        self.psiLOS = psiLOS
        self.ePsi = ePsi
        #Apenas armazena o ângulo desejado e o erro em radianos em variáveis do objeto para fins de registro e depuração.

        #3. Estimar a Velocidade de Rotação (O "D" do Controlador PD)
        alphaa = 0.2 #Define o "peso" do filtro de suavização.
        deg_usv = np.degrees(usv_psi) #Converte o ângulo atual para graus para facilitar os cálculos.
        #O termo abaixo é responsável por calcular a velocidade de rotação do ASV (o termo derivativo "D")
        if self.usv_psi_old > 90.0 and self.vPsi > 0 and deg_usv < 0.0:
            #self.usv_psi_old > 90.0: O ângulo antigo estava no quadrante superior (perto de 180°)?
            #self.vPsi > 0: O ASV estava girando no sentido positivo (horário)?
            #deg_usv < 0.0: O novo ângulo subitamente se tornou negativo?
                #Se todas as três condições são verdadeiras, o programa tem certeza que o salto de +180 para -180 aconteceu.
            usv_psi2 = deg_usv + 360.0
            #A Correção: Esta é a solução. Se o novo ângulo é, por exemplo, -179°, o código soma 360° a ele, resultando em 181°. Agora, o cálculo da velocidade será 181 - 179 = 2°, um valor pequeno e realista. O ângulo foi "desenrolado"
            self.newvPsi = self.vPsi * (1.0 - alphaa) + alphaa * ((usv_psi2 - self.usv_psi_old) / dt)
            #O cálculo da velocidade de rotação suavizada é feito usando o ângulo corrigido, usv_psi2.
            #E, crucialmente, o self.usv_psi_old para o próximo ciclo também é salvo com este valor corrigido para manter a continuidade.
            self.usv_psi_old = usv_psi2
        elif self.usv_psi_old < 90.0 and self.vPsi < 0 and deg_usv > 0.0:
            #O bloco elif faz exatamente a mesma coisa, mas para a direção oposta (o salto de -180° para +180°).
            usv_psi2 = deg_usv - 360.0
            self.newvPsi = self.vPsi * (1.0 - alphaa) + alphaa * ((usv_psi2 - self.usv_psi_old) / dt)
            self.usv_psi_old = usv_psi2
        else:
            #O bloco else é o caso normal, para quando o ASV está girando sem cruzar essa fronteira.
            self.newvPsi = self.vPsi * (1.0 - alphaa) + alphaa * ((deg_usv - self.usv_psi_old) / dt)
            self.usv_psi_old = deg_usv

        #4. Filtros de Segurança e Estabilização
        #Depois de obter um cálculo matematicamente correto da velocidade
        #o código realiza uma série de verificações para garantir que o valor seja fisicamente plausível e estável
        #A detecção por imagem pode ter pequenos "ruídos" ou falhas momentâneas, e esses filtros evitam que o controlador reaja a eles.
        #Filtro 1: Validade Numérica. A função np.isfinite() verifica se o resultado do cálculo é um número válido (não é "infinito" nem "NaN" - Not a Number)
        if not np.isfinite(self.newvPsi):
            #Se, por alguma razão (como dt sendo zero), o cálculo falhar, esta linha descarta o resultado inválido e mantém o valor de velocidade anterior.
            self.newvPsi = self.vPsi

        #Filtro 2: Detecção de Picos (Spikes). Este é um filtro inteligente contra ruído.
        #Ele mede a mudança absoluta entre a velocidade recém-calculada (newvPsi) e a velocidade antiga (vPsi).
        if abs(max([self.newvPsi, self.vPsi]) - min([self.newvPsi, self.vPsi])) > 10.0:
            #Se essa mudança for maior que 10 graus por segundo (um valor muito alto para uma mudança instantânea), o código assume que foi um pico de ruído na detecção da câmera e ignora o novo valor.
            self.newvPsi = self.vPsi #não aceita o novo valor (self.newvPsi) e mantém o valor antigo (self.vPsi)
        else:
            self.vPsi = self.newvPsi #aceita o novo valor e atualiza self.vPsi

        #Filtro 3: Saturação e Valor Final Seguro
        self.vPsi = max(min(self.vPsi, 30.0), -30.0) #Esta é a trava de segurança final. 
        #Não importa o que os cálculos digam, esta linha limita (grampeia) a velocidade de rotação para que ela nunca seja maior que 30 graus/s ou menor que -30 graus/s.
        vPsi_safe = self.vPsi if np.isfinite(self.vPsi) else 0.0 #Variável Final: Cria a variável vPsi_safe que será de fato usada na fórmula final do controlador PD.
        #Ela pega o valor de self.vPsi já filtrado e limitado. Como uma última redundância de segurança, se o valor ainda for inválido, ele usa 0.0.

        #5. A Lei de Controle e Formatação dos Comandos
        # PI/PID yaw controller (simplified)
        FpsiProp = (self.KPpsi * ePsi) - (self.KvPpsi * vPsi_safe) + (self.KIpsi * self.integralErrorPsi) #A Lei de Controle PD. Esta é a equação que define a "força de giro" (FpsiProp).
        #self.KPpsi * ePsi: Termo Proporcional.
        #- (self.KvPpsi * vPsi_safe): Termo Derivativo. O sinal negativo é o que o torna um "amortecedor", agindo contra a velocidade de rotação.
        #+ (self.KIpsi * self.integralErrorPsi): Termo Integral (desativado).
        if not np.isfinite(FpsiProp):
            #Mais uma verificação de segurança para garantir que a força calculada seja um número válido.
            FpsiProp = 0.0
        self.FpsiProp = float(FpsiProp)

        # saturate & bias (mantive sua formula)
        if self.FpsiProp > 0:
            self.FpsiProp = 0.7 * abs(self.FpsiProp) + 400.0
        elif self.FpsiProp < 0:
            self.FpsiProp = -(0.7 * abs(self.FpsiProp) + 400.0)
            #Transforma o resultado teórico do controlador em um comando prático.
            #O + 400.0 age como um "bias" para garantir que o motor receba energia suficiente para vencer a inércia, e o 0.7 * escala o resultado.

        # forward force proportional à distância
        dist_pix = np.hypot(usv_x - self.target[0], usv_y - self.target[1]) #O objetivo desta linha é calcular quantos pixels separam o ASV do alvo (dx, dy)
        #np.hypot(dx, dy): Esta é a função da biblioteca NumPy que aplica o Teorema de Pitágoras. Ela calcula a hipotenusa de um triângulo retângulo cujos catetos são dx e dy
        #O resultado é a distância em linha reta (a hipotenusa) entre o robô e o alvo.
        #O valor final, dist_pix, é um número de ponto flutuante (ex: 253.8_ pixels) que representa a distância euclidiana exata.
        fp = int(min(max(dist_pix * self.CMD_SCALE_FWD, 0), self.CMD_MAX))
        #lembrando que self.CMD_SCALE_FWD: É um fator de multiplicação. A força de avanço é calculada como distancia_em_pixels * CMD_SCALE_FWD. Se você aumentar esse valor, o ASV andará mais rápido quando estiver longe do alvo.
        #A força é diretamente proporcional à distância, ou seja, o ASV vai navegar mais rápido se estiver mais distante do alvo.
        #max(..., 0): Esta função garante que a força nunca seja negativa. Ela compara o resultado anterior (600.0) com 0 e retorna o maior dos dois.
        #min(..., self.CMD_MAX): Esta função garante que a força nunca exceda o limite máximo. Ela compara o resultado anterior (600.0) com o valor de self.CMD_MAX (que é 999).
        #int(...): A etapa final. converte o resultado final (600.0) para um número inteiro (600).
        'Atenção à linha abaixo'
        #self.FpProp = fp
        'se eu tirar o comentário da linha #self.FpProp = fp'
        'o comportamento do ASV mudará de uma velocidade de avanço constante para uma velocidade de avanço variável e proporcional à distância até o alvo.'
        'Vamos detalhar o que acontece:'

            #Com a linha comentada (Situação Atual)
                #A variável self.FpProp é inicializada com o valor 50 na função __init__.
                #Dentro da função propulsion, o valor de self.FpProp nunca é alterado.
                #O comando de avanço (speedFWD) é sempre calculado com base nesse valor fixo de 50.
                #Resultado: O ASV tenta se mover para frente com a mesma velocidade o tempo todo, não importa se está a 1 metro ou a 1 centímetro do alvo.

            #Com a linha descomentada (Nova Situação)
                #Ao remover o #, a linha self.FpProp = fp passa a ser executada a cada ciclo.
                #Primeiro, o código calcula a distância até o alvo:
                    #dist_pix = np.hypot(usv_x - self.target[0], usv_y - self.target[1])
                #Depois, calcula fp, que é essa distância multiplicada por uma escala:
                    #fp = int(min(max(dist_pix * self.CMD_SCALE_FWD, 0), self.CMD_MAX))
                #Agora, a linha self.FpProp = fp atualiza o valor da força de propulsão com base nesse fp recém-calculado.

        # waypoint switching
        'Se esse bloco fosse ativado, ele transformaria o ASV de um robô que apenas vai do ponto A ao ponto B'
        'para um robô que pode navegar por uma trajetória complexa: do ponto A para o B, depois para o C, depois para o D, e assim por diante, de forma totalmente automática.'
        '''reach_threshold = 5  # pixels
        if dist_pix <= reach_threshold:
            if self.waypoint < len(self.target_list) - 1:
                self.waypoint += 1
                self.target = [int(self.target_list[self.waypoint][0]), int(self.target_list[self.waypoint][1])]
                print(f"[WAYPOINT] avançou para {self.waypoint}, novo target {self.target}")
            else:
                self.FpProp = 0
                print("[WAYPOINT] último alvo alcançado. Parando forward.")'''

        mag_turn = int(min(max(int(abs(self.FpsiProp*2.5)), 0), self.CMD_MAX)) ##Pega o comando de giro self.FpsiProp, o escala (*2.5)
        #Pega seu valor absoluto, e o limita entre 0 e CMD_MAX (999). Este será o número enviado ao Arduino.
        # OBS: aqui eu mantenho a sua convenção original: '-' quando FpsiProp >=0 e '+' quando FpsiProp <0.
        # Se sua plataforma espera o contrário, troque a lógica do sinal aqui.
        sign = '-' if self.FpsiProp >= 0 else '+' #Define a direção do giro.
        #O sinal (+ ou -) será o primeiro caractere do comando de giro na mensagem final.
        speedTURN = sign + str(mag_turn).zfill(4) #Cria a string final para o comando de giro.
        #str(mag_turn) converte o número em texto e .zfill(4) garante que ele tenha 4 dígitos (ex: 550 vira "0550"). O resultado é algo 

        mag_fwd = int(min(max(int(abs(self.FpProp*4.5)), 0), self.CMD_MAX))
        speedFWD = str(mag_fwd).zfill(4)
        #Faz o mesmo para o comando de avanço, usando o valor de self.FpProp (seja ele o fixo ou o variavel com a distância). O resultado é uma string de 4 dígitos como "0225".

        self.timenow = now
        #Atualiza self.timenow com o tempo atual. Isso é crucial para que o cálculo de dt no próximo ciclo seja correto.
        return [speedFWD, speedTURN]
        #A função retorna os dois comandos como uma lista de strings, ['0225', '-0550'], prontos para serem montados e enviados ao ASV.

    def image_processor(self):
        #responsável por orquestrar tudo: desde a captura da imagem até o envio dos comandos.

        #1. Abertura da Câmera (A Parte Mais Complexa)
        '''Deve-se primeiro obter acessp à imagem da webcam, (Diferentes OS e diferentes câmeras usam tecnologias de software distintas para se comunicar.)
        O OpenCV lida com isso atráves de "Backends"'''

        ''' que são "Backends"? Pense nos backends como diferentes "idiomas" que o OpenCV pode usar para falar com os drivers da sua câmera.
        CAP_DSHOW (DirectShow) e CAP_MSMF (Media Foundation) são mais modernos e comuns no Windows, enquanto CAP_VFW (Video for Windows) é mais antigo.'''

        # prefer backends
        preferred_backends = [(cv2.CAP_DSHOW, "CAP_DSHOW"),(cv2.CAP_MSMF,  "CAP_MSMF"),(cv2.CAP_VFW,   "CAP_VFW"),]

        '''Por que uma lista? O código cria uma lista de backends preferidos para tentar em sequência.'''
        '''Se o primeiro "idioma" falhar, ele tenta o próximo. Isso torna o código mais robusto e com maior probabilidade de funcionar em diferentes computadores.'''

        cap = None
        used_backend_name = None
        for backend, name in preferred_backends:
            #for backend, name in preferred_backends: -> Este é um loop que vai iterar sobre a lista de backends que acabamos de criar.
            cap = cv2.VideoCapture(self.camera_index, backend)
            '''
            cap = cv2.VideoCapture(self.camera_index, backend): Esta é a linha mais importante. 
            cv2.VideoCapture() é a função do OpenCV que tenta "abrir" a câmera.
                self.camera_index: O número da câmera a ser usada (ex: 0 para a webcam integrada).
                backend: O "idioma" específico que ele tentará usar nesta iteração do loop.
                '''
            if cap is not None and cap.isOpened():
                '''
                if cap is not None and cap.isOpened(): -> Após a tentativa, esta linha verifica se a conexão foi bem-sucedida. 
                isOpened() retorna True se a câmera estiver pronta para enviar imagens.
                '''
                print(f"[OK] Opened camera index={self.camera_index} with backend={name}")
                used_backend_name = name
                break
                #break: Se a câmera for aberta com sucesso, o comando break interrompe o loop imediatamente. Não há necessidade de tentar os outros backends.
            else:
                '''else:: Se isOpened() retornar False, ele imprime uma mensagem de falha e o loop continua para tentar o próximo backend da lista.'''
                # 1.1. Informa o usuário sobre a falha específica
                print(f"[FAIL] Could not open index={self.camera_index} with backend={name}")
                
                # 1.2. Tenta liberar a câmera de forma segura
                try:
                    if cap is not None:
                        cap.release()

                    '''
                    Função: Esta é a parte mais importante. cap.release() é a função do OpenCV que libera a câmera.
                    Por que isso é essencial? Às vezes, mesmo que a câmera não seja aberta com sucesso (isOpened() é falso),
                    o sistema operacional pode ter alocado o recurso para o seu programa.
                    Se você não liberar explicitamente a câmera com cap.release(), ela pode ficar "presa" ou "bloqueada".
                    Se isso acontecer, nenhuma outra tentativa (nem mesmo com outros backends)
                    e nenhum outro programa conseguirá usar a câmera até que seu script seja finalizado, ou em casos piores, até que o computador seja reiniciado.
                    '''

                except:
                    pass

                '''
                Por que o try...except? O bloco try...except adiciona uma camada extra de segurança.
                Ele tenta executar o cap.release(), mas se por algum motivo essa ação gerar um erro,
                o except: pass simplesmente ignora o erro e impede que o programa inteiro trave. É uma programação defensiva.
                '''

                # 1.3. Reseta a variável para a próxima tentativa
                cap = None

                '''
                Função: Após tentar a limpeza, esta linha redefine a variável cap de volta para None.
                Por que isso é importante? Isso garante que, na próxima iteração do loop for,
                quando ele for tentar o próximo backend (ex: "CAP_MSMF"), a variável cap esteja "limpa" para receber o novo objeto VideoCapture.
                Também garante que, se todas as tentativas falharem, cap permanecerá como None,
                o que fará com que a verificação final após o loop funcione corretamente.
                '''

        if cap is None or not cap.isOpened():
            '''
            Após o loop, esta verificação final confere se a variável cap ainda é None (ou seja, se nenhum dos backends funcionou).
            Se for o caso, ele imprime uma mensagem de erro fatal e o return encerra a função image_processor, parando o programa.
            '''
            print("Erro: não foi possível abrir a câmera com os backends testados.")
            return

        #2. Preparação da Janela e do Detector de AprilTags

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

        #Esta linha cria a janela que aparecerá na sua tela para exibir a imagem da câmera.
            #'frame': É o nome (e título) da janela.
            #cv2.WINDOW_NORMAL: Permite que você redimensione a janela manualmente com o mouse.

        # Detector da AprilTag(pupil_apriltags)
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

            '''
            Este bloco try...except é outra medida de robustez. Ele tenta inicializar o detector de AprilTags usando a biblioteca preferida, pupil_apriltags.
            detector = Detector(...): Esta linha cria o objeto detector. 
            Os parâmetros internos são otimizações para ajustar a performance e a precisão da detecção. Por exemplo:
            families="tag36h11": Especifica o "dicionário" de tags que ele deve procurar. A tag36h11 é uma família muito comum em robótica.
            nthreads=2: Usa dois núcleos do seu processador para acelerar a detecção.
            except Exception: -> Se a biblioteca pupil_apriltags não estiver instalada ou falhar por algum motivo, o código não trava.
            Em vez disso, ele pula para o bloco except e tenta usar uma biblioteca alternativa (apriltag). 
            Isso aumenta a chance de o código rodar mesmo que o ambiente do usuário não seja exatamente o esperado.
            '''

        #3. Configuração da Rede e Início do Loop Principal

        # independent timing para full-frame e warp-frame
        detection_interval_full = 0.20
        detection_interval_warp = 0.20
        last_detect_time_full = 0.0
        last_detect_time_warp = 0.0

        '''
        Estas variáveis de tempo (detection_interval... e last_detect_time...) são parte do código original que foi comentado.
        A ideia era não rodar a detecção de AprilTags em todos os frames, mas apenas a cada 0.20 segundos para economizar processamento. 
        No estado atual do código, elas não têm efeito.
        '''

        print(f"Starting main loop (camera index={self.camera_index}, backend={used_backend_name}, rotate={self.rotate})")
        # ajuste o HOST e PORT para o IP do seu robo
        PORT = 2222 #do ESP8266
        # HOST = '192.168.0.80'  #Robo 1 -> endereço IP do robô 1
        HOST = "192.168.0.83" #Robo 2 -> endereço IP do robô 2
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = (HOST, PORT)

        '''
        Configuração do Endereço: Define o HOST (o endereço IP do seu ASV na rede Wi-Fi) e a PORT (a porta na qual o ESP8266 está "ouvindo").
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM): Esta linha cria o objeto de comunicação.
            socket.AF_INET: Especifica que estamos usando uma rede IP (o padrão da internet e Wi-Fi).
            socket.SOCK_DGRAM: Especifica que estamos usando o protocolo UDP.
            UDP é como enviar uma carta: rápido, mas sem garantia de entrega. 
            É ideal para este tipo de aplicação onde enviar o comando mais recente rapidamente é mais importante do que garantir que todos os comandos antigos chegaram.
        dest = (HOST, PORT): Junta o IP e a porta em uma tupla, que é o formato que a função de envio de dados espera.
        '''

        try:
            while True:
                #try...while True:: Inicia o loop principal do programa, 
                #que rodará infinitamente até ser interrompido (Ctrl+C ou a tecla 'q'). O try é para capturar essas interrupções de forma elegante.
                ret, frame = cap.read() #Esta é a função que efetivamente captura uma nova imagem da câmera.
                '''
                    frame: Contém a imagem em si.
                    ret: É uma variável booleana (True/False) que indica se a captura foi bem-sucedida.
                '''
                if not hasattr(self, '_printed_frame_info'):
                    print("DEBUG frame read:", ret, "shape:", None if frame is None else getattr(frame, "shape", None))
                    self._printed_frame_info = True
                if not ret or frame is None:
                    time.sleep(0.05)
                    continue
                '''
                    if not ret or frame is None:: Se a captura falhar (ret for False), o código espera um pouco (time.sleep)
                    e usa continue para pular para a próxima iteração do loop, tentando capturar um novo frame.
                '''

                # rotaciona referências somente 1x, baseado nas dimensões originais (no código original o que estava comentado ainda está)
                #Este trecho do código é a primeira etapa do processamento de imagem dentro do loop principal.
                #A missão dele é preparar a imagem capturada pela câmera para que o detector de AprilTags possa trabalhar nela da forma mais eficiente possível.

                #4. O Bloco Comentado: Rotação dos Pontos de Referência
                
                '''
                O bloco comentado abaixo faz parte de uma parte mais avançada do código, a correção da perspectiva (warp)
                Fixa-se 4 AprilTags nos cantos da área de teste de posição (self.reference)
                para distorcer a imagem da câmera e criar uma visão perfeitamente retangular de cima
                '''

                '''
                if not self._refs_rotated and self.rotate in (90,180,270): # Garante que a correção só acontecerá uma única vez (no ínicio do programa)
                    h0, w0 = frame.shape[:2]  # dimensões originais da imagem
                    new_refs = []
                    for rp in self.references: # passa a posição de cada uma das quatro AprilTags
                        new_refs.append(self.rotate_point(rp, w0, h0)) #-> Usa a função rotate_point (Linha 119 quando escravo isso) para calcular a nova posição de cada tag
                    self.references = new_refs #atualiza a lista de posição das 4 tags com as novas posições após a rotação
                    self._refs_rotated = True
                    print("[INFO] references rotacionadas para a nova orientação.")
                '''

                #5. Rotação do Frame e Conversão para Tons de Cinza

                # agora rotaciona frame conforme pedido
                frame = self.rotate_frame(frame) #usa a função rotate_frame para e gira com base no valor de self.rotate (Linha 102)
                #Garante a orientação correto do resto do programa 

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #cv2.cvtColor significa "converter cor".
                #Ela pega o frame colorido (que no OpenCV está no formato BGR - Blue, Green, Red). 
                #Usa a constante cv2.COLOR_BGR2GRAY para converter os três canais de cor (B, G, R) de cada pixel em um único valor de brilho (de 0 a 255) (Cinza-gray).
                #Por que é importante? Os algoritmos de detecção de AprilTags (e muitos outros em visão computacional) são projetados para trabalhar com imagens em tons de cinza.

                #6. A Detecção da AprilTag

                #now = time.time()
                results_full = []
                '''
                if now - last_detect_time_full >= detection_interval_full:
                    try:
                        t0 = time.time()
                        '''
                
                '''
                O bloco "if now - ..." era uma otimização , o detector.detect (que detecta a tag) não ia rodar todos os frames, mas somente a cada
                detection_interval_full (20seg) para economizar processamento, mas isso sacrificaria controle responsivo do robô
                DEIXAR DESATIVADO
                '''

                results_full = detector.detect(gray) #detector é o objeto que criamos anteriormente, pronto para procurar por tags tag36h11.
                print(results_full)
                #O método .detect(gray) executa o algoritmo de detecção sobre a imagem em tons de cinza.
                '''O algoritmo faz várias coisas complexas: procura por regiões de alto contraste, identifica contornos de quadrados (quads),
                analisa o padrão de pixels preto e branco dentro desses quadrados, e os compara com a "biblioteca" de tags tag36h11.'''
                #results_full: O resultado é uma lista. Se nenhuma tag for encontrada, a lista estará vazia. 
                #Se uma ou mais tags forem encontradas, a lista conterá um objeto para cada tag, com todas as informações sobre ela 
                #(ID da tag, coordenadas do centro, coordenadas dos 4 cantos, etc.).

                '''        
                dt = time.time() - t0
                        if dt > 0.5:
                            print(f"Detector full-frame lento: {dt:.2f}s")
                    except Exception as e:
                        print("Detector full-frame error:", e)
                        traceback.print_exc()
                        results_full = []
                    last_detect_time_full = now
                    '''
                
                #pegar o que foi dissertado sobre o bloco acima do pc em casa

                #7. A Transformação de Perspectiva (O "Warp")
                #Correção de Perspectiva ou "Bird's-Eye View" (Visão de Pássaro).

                # tenta perspectiva (warp) se referências válidas
                '''dst = None #imagem de destino
                colored = None #versão colorida da dst
                results_warp = [] #resultados da detecção na imagem transformada (zerada a cada ínicio de loop)
                self._warp_wh = None

                if all(isinstance(ref, (list,tuple)) and len(ref)==2 for ref in self.references):
                    #a linha acima confere se a variável self.references (que deveria conter a posição das 4 AprilTags nos cantos do tanque está preenchida corretamente)
                    #Caso não, pula todo o bloco abaixo
                    
                    try:
                        pts1 = np.float32(self.references)
                        #pts1 converte a lista de pontos das 4 AprilTags de referência (que estão na imagem distorcida da câmera)
                        #para um formato que o OpenCV entenda. Serão os 4 cantos do 'trapezio' que o código corrige
                        
                        warp_scale = 5
                        w = int(self.area[0] * warp_scale)
                        h = int(self.area[1] * warp_scale)
                        pts2 = np.float32([[0, 0], [0, h], [w, h], [w, 0]]) #Pontos de Destino (pts2): Aqui, o código define os 4 cantos do retângulo de destino.
                        #Ele pega as dimensões reais do tanque (self.area) e multiplica por um fator de escala (warp_scale) para criar uma imagem de saída em tamanho razoavel
                        #pts2 representa os 4 cantos dessa imagem retangular de destino: [canto superior esquerdo, canto inferior esquerdo, canto inferior direito, canto superior direito]
                        
                        M = cv2.getPerspectiveTransform(pts1, pts2) #A Matriz de Transformação (M): Esta é uma função do OpenCV. 
                        #cv2.getPerspectiveTransform compara os 4 pontos de origem (pts1) com os 4 pontos de destino (pts2) e calcula uma matriz de transformação (M).
                        #Essa matriz contém toda a informação matemática necessária para "esticar" e "distorcer" a imagem original para que os pontos de pts1 se alinhem perfeitamente com os pontos de pts2.
                        
                        dst = gray #cv2.warpPerspective(gray, M, (w, h)) #A linha cv2.warpPerspective(gray, M, (w, h)) (que está comentada, mas deveria estar ativa) 
                        #aplicaria a matriz M à imagem gray original, gerando uma nova imagem dst que é a visão "de cima". 
                        
                        colored = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
                        #Cria uma versão colorida da imagem dst para que se possa desenhar linhas e círculos coloridos nela.

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

                #8. Processamento na Imagem Transformada ("Warped")

                'Depois de criar a imagem dst (a visão de cima), 
                o código faria toda a detecção e controle dentro deste novo "mapa", onde as coordenadas são muito mais precisas.'

                # ===== PROCESSA DETECÇÕES NO WARP (preferido para posições mais precisas) =====
                if results_warp:
                    for r in results_warp:
                    #if results_warp: -> Se alguma AprilTag for encontrada na imagem transformada (dst), este bloco é executado.
                        try:
                            (ptA, ptB, ptC, ptD) = r.corners
                            ptB = (int(ptB[0]), int(ptB[1]))
                            ptC = (int(ptC[0]), int(ptC[1]))
                            ptD = (int(ptD[0]), int(ptD[1]))
                            ptA = (int(ptA[0]), int(ptA[1]))
                            (cX, cY) = (int(r.center[0]), int(r.center[1]))

                            # desenha target (no sistema warp)
                            cv2.circle(colored, (self.target[0], self.target[1]), 60, (0, 255, 255), 2) # TARGET warp
                            #Agora, todos os desenhos (o alvo, os eixos da tag, etc.) são feitos na imagem colored, que é a visão "de cima". 
                            #Isso é muito melhor porque um círculo desenhado nela será sempre um círculo, e não uma elipse distorcida.

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

                                #O mais importante é que as coordenadas (cX, cY) e angle da AprilTag do ASV 
                                #agora estão no sistema de coordenadas do "mapa". O self.target também está definido neste mesmo sistema.

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
                
                '''
                Honestamente, não entendi quase nada desse bloco todo comentado ai em cima
                Mas,
                parece que ele basicamente faz:
                1.Encontra 4 marcadores fixos na arena.
                2.Usa-os para criar uma visão "de cima" (um mapa 2D) da arena.
                3.Localiza e controla o ASV dentro deste mapa, o que elimina erros causados pela perspectiva da câmera.
                '''

                # ===== PROCESSA DETECÇÕES NO FULL FRAME (fallback) =====

                '''
                O nome "Full-Frame" significa que todos os cálculos são feitos diretamente na imagem "cheia" (completa) que vem da câmera, 
                sem as correções de perspectiva feitas no bloco comentado anterior.
                '''

                #1. Preparação e Início do Loop

                #O processo começa logo após a linha results_full = detector.detect(gray). 
                #Neste ponto, já temos a lista results_full que contém todas as AprilTags (Todas mesmo, dos ASV e do tanque) que o detector encontrou.

                gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) #Esta linha converte a imagem em tons de cinza (gray) de volta para uma imagem colorida (gray_bgr).
                #Aparentemente isso é feito pois o OpenCV só pode desenhar formas coloridas (como circulas e linhas) em uma imagem colorida.
                #a imagem gray_bgr NÃO terá as cores originais detectadas pela câmera, mas terá as 3 necessárias (o RGB) necessária para o OpenCV adicionar cores a ela.

                if results_full:
                    #checa se a lista results_full não está vazio. Caso esteja vazia pula a condição é False, e todo o bloco a seguir é ignorado.
                    for r in results_full:
                        #Se uma ou mais tags foram encontradas, este loop irá executar o bloco de código seguinte uma vez para cada tag detectada. 
                        #A variável r se torna o objeto que contém todas as informações da tag atual (seu ID, cantos, centro, etc.).

                        try:
                            #Inicia um bloco de tratamento de exceções. O processamento de dados de imagem pode, ocasionalmente, gerar erros inesperados.
                            #O try permite que o programa tente executar o código seguinte, mas se um erro ocorrer, ele não irá travar (except no final).

                            #2. Extração dos Dados da AprilTag

                            (ptA, ptB, ptC, ptD) = r.corners #Lembrando que r é a a tag detectada que contém um atributo, o corners.
                            #Ele contém as coordenadas (x,y) dos quatro cantos da tag.
                            #A linha de código "desempacota" essas coordenadas em quatro variáveis separadas: ptA, ptB, ptC e ptD. Em sentido horário.

                            ptB = (int(ptB[0]), int(ptB[1]))
                            #As coordenadas dos cantos obtidas anteriormente vem como números de ponto flutuante, por exemplo [250.73, 180.12]
                            #Mas no OpenCV as funções de desenho exigem coordenadas como numeros inteiros
                            #Aqui converte-se as duas coordenadas (x[0], y[1]) de flutuantes para inteiros.
                            ptC = (int(ptC[0]), int(ptC[1]))
                            ptD = (int(ptD[0]), int(ptD[1]))
                            ptA = (int(ptA[0]), int(ptA[1]))

                            (cX, cY) = (int(r.center[0]), int(r.center[1])) #Similarmente, esta linha pega as coordenadas do centro da tag (r.center), que também são de ponto flutuante
                            #E as converte para inteiros cX (centro X) e cY (centro Y). Esta é a posição oficial do ASV na imagem.

                            #3. Lógica de Controle e Feedback Visual

                            #Agora que temos a posição, o código decide o que fazer e desenha informações úteis na tela.

                            ##if r.tag_id==...: Estas linhas comentadas faziam parte da funcionalidade de correção de perspectiva.
                            # Elas teriam a função de identificar as tags de referência (IDs 1, 7, 3, 4) e atualizar suas posições, 
                            #Mas como a funcionalidade está desativada, elas foram comentadas.

                            #Atualizar caso as tags de referência que limitem o tanque mudarem. 1...4 e etc 
                            #if r.tag_id==1: self.references[0]=[cX,cY]
                            #if r.tag_id==7: self.references[1]=[cX,cY]
                            #if r.tag_id==3: self.references[2]=[cX,cY]
                            #if r.tag_id==4: self.references[3]=[cX,cY]
                            #print()

                            # se não for referencia, considera robo e comanda
                            if 1==1:#r.tag_id not in [1,7,3,4]:
                                '''
                                if 1==1:: Esta é uma maneira simples de garantir que o código dentro do if sempre execute. 
                                O código original r.tag_id not in [1,7,3,4] servia para ignorar as tags de referência e só controlar a tag do robô. 
                                Como a funcionalidade de referência está desativada, 
                                1==1 (que é sempre True) serve como um "foda-se" para garantir que qualquer tag detectada seja tratada como o robô.
                                '''

                                '''Desenho da tag'''

                                cv2.line(gray_bgr, (cX, cY), (int((ptB[0]+ptC[0])/2), int((ptB[1]+ptC[1])/2)), (0, 0, 255), 2) #Desenho dos Eixos da Tag
                                #cv2.line(...): Desenha uma linha na imagem gray_bgr
                                #de: (cX, cY) (o centro da tag) para: (int((ptB[0]+ptC[0])/2), int((ptB[1]+ptC[1])/2)) 
                                #(o ponto médio entre os cantos B e C). Esta linha representa o eixo X local do robô (para frente).
                                #(0, 0, 255): A cor da linha em formato BGR. (Aqui sendo vermelho). 2: A espessura da linha em pixels.

                                cv2.line(gray_bgr, (cX, cY), (int((ptA[0]+ptB[0])/2), int((ptA[1]+ptB[1])/2)), (0, 255, 0), 2) #Desenha o outro eixo da tag
                                #Do centro até o ponto médio entre A e B. Esta representa o eixo Y local do robô. A cor é (0, 255, 0), ou seja, verde.

                                cv2.circle(gray_bgr, (cX, cY), 5, (255, 0, 0), -1) #Desenha um círculo no centro da tag (cX, cY) com raio de 5 pixels. 
                                #A cor é (255, 0, 0) (azul) e a espessura -1 significa que o círculo será preenchido

                                '''Desenho do alvo'''

                                tx = int(self.target[0])
                                ty = int(self.target[1])
                                cv2.circle(gray_bgr, (tx, ty), 8, (0,255,255), 2)
                                cv2.putText(gray_bgr, f"Target:{self.target}", (tx+10, ty+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1,2)
                                #Estas linhas desenham o alvo (self.target) na tela. 
                                #cv2.circle desenha um círculo amarelo no local do alvo, 
                                #e cv2.putText escreve o texto "Target:[x,y]" ao lado dele para fácil visualização.

                                angle = atan2(int((ptB[1]+ptC[1])/2)-cY, int((ptB[0]+ptC[0])/2)-cX) #Calcula o ângulo de orientação do ASV em radianos.
                                #A lógica é calcular o ângulo do vetor que vai do centro da tag (cX, cY) até o ponto médio do seu lado frontal (o mesmo ponto usado para desenhar a linha vermelha).
                                
                                tgtR = self.target_alg(cX, cY) #Chama a função target_alg para calcular o vetor de direção unitário que aponta do robô (cX, cY) para o alvo.
                                #Na função cX e cY viram, respectivamente, usv_x, usv_y.
                                 
                                speedR = self.propulsion(cX, cY, angle, tgtR[0], tgtR[1]) #Chama a função propulsion para calcular os comandos do motor [speedFWD, speedTURN] na variável speedR
                                #Esta é a chamada principal, onde toda a lógica do controlador PD é executada. 
                                #Ela recebe a posição (cX e cY) e o ângulo atuais (angle), e a direção desejada (tgtR), e retorna os comandos de motor
                                #Dentro da função eles viram, respectivamente, usv_x, usv_y, usv_psi, g10, g20
                                """ Voltando para o def propulsion:
                                usv_x, usv_y : posição do robô no sistema (pixels do warp)
                                usv_psi : orientação atual do robô (radianos, como você passa com atan2)
                                g10,g20 : direção unitária para o target (vx,vy)
                                Retorna [speedFWD, speedTURN] strings (formato zfilled).
                                """

                                '''
                                Esta última parte é sobre agir e comunicar: pegar todos os cálculos complexos que fizemos, 
                                transformá-los em ações (enviar comandos), fornecer feedback visual para o usuário (desenhar na tela) 
                                e garantir que o programa possa ser encerrado de forma limpa e segura.
                                '''

                                #1. Exibição de Telemetria (Feedback Visual)

                                '''
                                Depois que a propulsion retorna os comandos, o programa cria um "painel de controle" de texto diretamente na imagem da câmera. 
                                Isso é extremamente útil para você, o operador, entender o que o robô está "pensando" em tempo real.
                                '''

                                info_usv = ["x: "+str(int(cX)),
                                            "y: "+str(int(cY)),
                                            "yaw: "+str(int(np.degrees(angle))),
                                            "yaw_rate: "+str(int(self.vPsi)),
                                            "force yaw: "+speedR[1],
                                            "time: "+str(time.time())]
                                #info_usv = [...]: Esta linha cria uma lista de strings. Cada string é uma peça de informação de telemetria que será exibida na tela.
                                    #"x: "+str(int(cX)) e "y: ...": Mostra as coordenadas de posição (inteiras) da AprilTag.
                                    #"yaw: ...": Mostra a orientação atual do robô em graus.
                                    #"yaw_rate: ...": Mostra a velocidade de rotação suavizada (self.vPsi), que é o resultado do termo Derivativo. Isso te diz o quão rápido o robô está virando.
                                    #"force yaw: "+speedR[1]: Mostra o comando de giro bruto (+0550, -0400, etc.) que está sendo enviado. speedR[1] é o comando de giro retornado pela função propulsion.
                                    #"time: ...": Mostra o tempo do sistema, útil para verificar se o programa está travado.

                                cnt_txt = 50
                                for txt in info_usv:
                                    cv2.putText(gray_bgr, txt, (10, cnt_txt), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1,2)
                                    cnt_txt+=15

                                #cnt_txt = 50: Inicializa uma variável que controlará a posição vertical (Y) do texto na tela, começando no pixel 50 a partir do topo.
                                    #for txt in info_usv:: Inicia um loop que passará por cada string da lista info_usv.
                                    #cv2.putText(...): Esta é a função do OpenCV para desenhar texto em uma imagem.
                                        #gray_bgr: A imagem onde o texto será desenhado. txt: A string de texto a ser escrita (ex: "x: 219").
                                        #(10, cnt_txt): As coordenadas (x, y) onde o texto começará. 10 pixels da borda esquerda, e cnt_txt pixels do topo. 
                                        #cv2.FONT_HERSHEY_SIMPLEX: O tipo da fonte.
                                        #0.5: O tamanho da fonte. (0,0,255): A cor do texto (vermelho). 1,2: A espessura da linha da fonte e o tipo de linha.
                                    #cnt_txt+=15: Após desenhar uma linha de texto, esta linha aumenta a posição vertical em 15 pixels. 
                                    #Isso garante que a próxima informação seja escrita logo abaixo da anterior, sem sobreposição.

                                #2. Montagem e Envio do Comando UDP

                                    '''Aqui, os comandos calculados são formatados e enviados pela rede para o ASV.'''

                                msg3 = "{"+speedR[1]+speedR[0]+"}"
                                #msg3 = ...: Monta a string final do comando.
                                #speedR[1] é o comando de giro (ex: "-0850") e speedR[0] é o de avanço (ex: "0225"). 
                                #A linha os concatena no formato que o código Arduino espera: {-08500225}.

                                print(f"[FULL] tag_id={r.tag_id} at ({cX},{cY}) angle={np.degrees(angle):.1f}deg -> msg SEND: {msg3} -> {dest}")
                                #print(...): Imprime a linha de status completa no seu terminal. 
                                #Esta é a sua principal ferramenta de depuração para ver o que o programa está fazendo a cada detecção.

                                if self.udp_ok:
                                #if self.udp_ok:: Uma pequena trava de segurança. Se um envio UDP falhar, self.udp_ok se torna False, 
                                #e o programa para de tentar enviar dados para não poluir o terminal com erros repetidos.
                                    try:
                                        udp.sendto(msg3.encode(), dest)
                                    except Exception as e:
                                        print("UDP send error:", e)
                                        self.udp_ok = False
                                        #try...except: Outro bloco de segurança. Ele tenta enviar o comando.
                                            #udp.sendto(msg3.encode(), dest): A ação final.
                                                #msg3.encode(): Converte a string de texto do comando em uma sequência de bytes, que é o formato que as redes entendem.
                                                #udp.sendto(..., dest): Envia esses bytes via UDP para o destino (dest), que contém o endereço IP e a porta do ASV.

                        except Exception:
                            #Ela funciona em conjunto com um bloco try: que vem antes (e que envolve todo o código de processamento da AprilTag).
                            # A lógica é: "Tente (try) executar o código de processamento. 
                            #Se qualquer tipo de erro (Exception) acontecer, em vez de travar, pule para este bloco except."
                            traceback.print_exc()
                            '''
                            O que faz? Esta é a ferramenta de diagnóstico. Se um erro for capturado, 
                            esta função da biblioteca traceback imprime no seu terminal um relatório completo e detalhado do erro. 
                            Esse relatório, chamado de "traceback" (ou "pilha de execução"), mostra:
                                O tipo de erro (ex: ZeroDivisionError, ValueError).
                                O arquivo e a linha exata onde o erro ocorreu.
                                A sequência de chamadas de função que levaram ao erro.
                            Por que é importante? Sem essa linha, você saberia que um erro aconteceu, mas não teria ideia do que foi ou onde procurar. 
                            traceback.print_exc() é a ferramenta mais útil para um programador entender e corrigir bugs.
                            '''
                            continue
                        '''
                            O que faz? continue é uma instrução que afeta o loop (for r in results_full:) em que está inserida. 
                            Ela diz ao programa: "Pare imediatamente esta iteração do loop e pule para a próxima".
                                Por que é importante? Imagine que a câmera detectou duas AprilTags, mas a primeira tem um defeito que causa um erro de cálculo. 
                                Sem o continue, o programa travaria ao processar a primeira tag. Com o continue, o programa faria o seguinte:
                                    Tenta processar a primeira tag. - > Ocorre um erro. -  > Entra no bloco except.
                                    Imprime os detalhes do erro com traceback.print_exc().
                            O continue é executado, e o programa ignora o resto do código para a primeira tag e pula para a próxima iteração, 
                            começando a processar a segunda tag normalmente.
                            '''

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
                    #if self.show_img:: Verifica a variável que foi configurada no início (show_window=True). Se for True...
                    cv2.imshow('frame', gray_bgr)
                        #cv2.imshow('frame', gray_bgr): ...esta função do OpenCV pega a imagem gray_bgr (com todos os desenhos e textos) 
                        #e a exibe na janela chamada 'frame' que foi criada no início. Esta é a linha que faz a janela da câmera aparecer na sua tela.

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Saindo por tecla 'q'")
                    break
                    #cv2.waitKey(1): Esta é uma função crucial para o OpenCV. Ela faz duas coisas: 
                    #1) Mostra a imagem na tela e espera por 1 milissegundo. 
                    #2) Se uma tecla for pressionada durante esse tempo, ela retorna o código da tecla.
                        #& 0xFF == ord('q'): Esta parte verifica se a tecla pressionada foi a letra 'q'.
                        #break: Se a tecla 'q' for pressionada, o comando break interrompe o loop while True, fazendo o programa sair do ciclo principal e continuar para o código de finalização.

        except KeyboardInterrupt:
            print("Interrompido pelo usuário (Ctrl+C).")
        except Exception as e:
            print("Erro inesperado:")
            traceback.print_exc()
        finally:
            #finally:: Este bloco de código é garantido a ser executado no final, não importa como o programa saiu do loop try (seja por break, por erro, ou por Ctrl+C).
            try:
                cap.release()
                #cap.release(): Libera a câmera, desligando-a e permitindo que outros programas a usem. Esta é uma das linhas mais importantes para a limpeza.
            except:
                pass
            cv2.destroyAllWindows()
            #cv2.destroyAllWindows(): Fecha todas as janelas que o OpenCV abriu.
            print("Camera and windows released/closed.")
            #print(...): Informa ao usuário que o encerramento foi bem-sucedido.


if __name__ == '__main__':
    #if __name__ == '__main__':: Esta é uma convenção padrão em Python. 
    #Ela significa: "Se este script está sendo executado diretamente (e não importado por outro script), então execute o código abaixo".
        # rotate=90 para girar 90° clockwise se seu dispositivo precisar
        #Image = main_class(camera_index=0 determina qual a câmera que o programa vai tentar usar. 0 -> padrão do pc/note , 1-> camêra usb externa
    Image = main_class(camera_index=0, show_window=True, rotate=0)
    #Image = main_class(...): Cria uma instância (um objeto) da sua classe main_class, passando os parâmetros de configuração e executando o __init__.
    Image.image_processor()
    #Image.image_processor(): Chama a função image_processor no objeto recém-criado, dando início a todo o processo que acabamos de estudar.