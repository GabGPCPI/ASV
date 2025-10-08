# /usr/bin/env python3

'''nessa versão foi adicionado o self.mission_complete = False # Flag para saber se a missão terminou, o self.FpProp = fp foi ativado e o BLOCO DE WAYPOINT SWITCHING foi modificado para parar de enviar comando quando o ASV chega ao alvo final'''

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
        self.target_list = [[220,340,0], [350,240,0]] #Uma lista que contém as coordenadas de todos os alvos que o ASV deve seguir em sequência.
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
        self.mission_complete = False # Flag para saber se a missão terminou 

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
        self.FpProp = fp
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

                        if self.mission_complete: #*(?)
                            continue # Pula para a próxima iteração do loop, ignorando todo o código abaixo

                        try:
                            (ptA, ptB, ptC, ptD) = r.corners
                            ptB = (int(ptB[0]), int(ptB[1]))
                            ptC = (int(ptC[0]), int(ptC[1]))
                            ptD = (int(ptD[0]), int(ptD[1]))
                            ptA = (int(ptA[0]), int(ptA[1]))
                            (cX, cY) = (int(r.center[0]), int(r.center[1]))

                            # waypoint switching
                            'Se esse bloco fosse ativado, ele transformaria o ASV de um robô que apenas vai do ponto A ao ponto B'
                            'para um robô que pode navegar por uma trajetória complexa: do ponto A para o B, depois para o C, depois para o D, e assim por diante, de forma totalmente automática.'
       
                            # --- INÍCIO DO BLOCO DE WAYPOINT SWITCHING --- LUGAR AJUSTADO PARA DENTRO DO PROCESSAMENTO DE IMAGEM ---

                            dist_pix = np.hypot(cX - self.target[0], cY - self.target[1])
                            reach_threshold = 5 

                            reach_threshold = 5  # pixels
                            if dist_pix <= reach_threshold:
                                if self.waypoint < len(self.target_list) - 1:
                                    self.waypoint += 1
                                    self.target = [int(self.target_list[self.waypoint][0]), int(self.target_list[self.waypoint][1])]
                                    print("------------------------------------------------------")
                                    print(f"ASV alcançou o waypoint {self.target}. avançando para {self.waypoint}, novo alvo {self.target}")
                                    print("------------------------------------------------------")
                                else:
                                    # Se for o último waypoint, para o robô e ativa a flag.
                                    if not self.mission_complete: # Garante que a mensagem só seja impressa uma vez
                                        print("---------------------------------------------------------")
                                        print("O ASV chegou ao último alvo alcançado. Missão concluída.")
                                        print("---------------------------------------------------------")
                                        self.FpProp = 0
                                        self.mission_complete = True 
        
                                #reach_threshold = 5 -> Esta linha define um raio de tolerância ao redor do alvo. reach_threshold (limite de alcance) é a distância em pixels que o programa considera como "ter chegado" ao alvo.
                                #if dist_pix <= reach_threshold: Esta é a condição principal. A cada ciclo da função propulsion, o código verifica a distância atual do ASV até o alvo (dist_pix).
                                #Se essa distância for menor ou igual ao limite de 5 pixels, o código dentro deste if é executado.
                                    #Dentro do primeiro if há outro if que faz: verifica se ainda existem mais alvos na lista (self.target_list) para seguir.
                                    #self.waypoint é o índice do alvo atual dentro da target_list
                                    #len(self.target_list) - 1: Calcula o índice do último item da lista. A condição self.waypoint < ... verifica se o índice do alvo atual (self.waypoint) ainda não é o último.
                                    #Se a condição for verdadeira (não é o último alvo): self.waypoint += 1: O índice é incrementado. Se ele estava indo para o alvo 0, agora ele vai para o alvo 1.
                                    #self.target = ...: A variável self.target (que é usada em todos os cálculos de navegação) é atualizada com as coordenadas do novo waypoint da lista.        

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
    #Image = main_class(camera_index=0 determina qual a câmera que o programa vai tentar usar. 0 -> padrão do pc/note , 1-> camêra usb externa
    Image = main_class(camera_index=1, show_window=True, rotate=0)
    Image.image_processor()