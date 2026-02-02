import numpy as np

class Controle_de_Alocação:
    def __init__(self):
        print("Iniciando o sistema de alocação do controle (fase 1)")

        #1. Definição da Geometria
        '''1 = Frente-Esquerda , 2 = Frente-Direita , 3 = Traseira-Esquerda , 4 = Traseira-Direita

        Com todos os propulsores simetricos à 45º da horizontal ou vertical têm-se:
        Prop1 (Frente-Esq): Jato sai p/ Frente-Dir -> Força p/ Trás-Esq (alpha = 225º)
        Prop2 (Frente-Dir): Jato sai p/ Frente-Esq -> Força p/ Trás-Dir (alpha = 135º)
        Prop3 (Trás-Esq): Jato sai p/ Trás-Dir     -> Força p/ Frente-Esq (alpha = 315º)
        Prop4 (Trás-Dir): ato sai p/ Trás-Esq      -> Força p/ Frente-Dir (alpha = 45º)'''
        self.alpha_deg = np.array([225, 135, 315, 45])
        self.alpha = np.radians(self.alpha_deg)

        '''Coordenadas dos propulsores em relação ao centro de massa obtido no Fusion360
        # Formato: [x (metros), y (metros)]
        # x+ = Frente, x- = Trás
        # y+ = Direita, y- = Esquerda'''
        self.motor_coords = np.array([
            [0.09792375 , -0.07528369], #Prop1 (Frente-Esq)
            [ 0.0979239,  0.07670449], #Prop2 (Frente-Dir)
            [-0.0815487, -0.09728376], #Prop3 (Trás-Esq)
            [-0.08154855,  0.09870441]  #Prop4 (Trás-Dir)
        ])

        num_motores_ang = len(self.alpha)
        num_motores_pos = len(self.motor_coords)

        if num_motores_ang != 4 or num_motores_pos != 4: 
            erro_mensagem = f"Erro: Geometria inserida é consistente! Ângulos: {num_motores_ang}, Coords: {num_motores_pos}."
            print(erro_mensagem)
            raise ValueError(erro_mensagem)
        
        if num_motores_ang != num_motores_pos:
            erro_mensagem = f"Erro: Número de ângulos diferente do número de coordenadas."
            print(erro_mensagem)
            raise ValueError(erro_mensagem)
        
        print("f-> Geometria Funcional: {num_motores_pos} propulsores configurados.")

        #2 Calculo da Matriz B
        try: 
            #Extração de vetores para cálculo vetorial
            prop_x = self.motor_coords[:, 0]
            prop_y = self.motor_coords[:, 1]

            #Cálculo do Braço de Momento (Mz)
            '''Mz = x*Fy - y*Fx (prod. vetorial)
            usa-se sin/cos dos ângulos para projetar a força'''
            momento_arms = prop_x * np.sin(self.alpha) - prop_y * np.cos(self.alpha)

            #Matriz de Configuração (3DOF)
            '''Linha 0: Força em Surge (x)
            Linha 1: Força em Sway (y)
            Linha 2: Momento em Yaw (N)'''
            self.B = np.array([
                np.cos(self.alpha), #Contribuição em X
                np.sin(self.alpha), #Contribuição em Y
                momento_arms        #Contribuição no Giro
            ])

            #Cálculo da matriz Pseudo-Inversa
            self.B_pinv = np.linalg.pinv(self.B, rcond = 1e-5) #rcond=1e-5 para ignorar valores muito pequenos (erro numérico)

            print("-> Matriz de Alocação calculada e invertida")
            # Debug: Mostrar a matriz para conferência visual
            print("Matriz B:\n", np.round(self.B, 2))

        except np.linalg.LinAlgError as e:
            print("Erro matemático: A matriz de configuração é singular")
            print("Isso significa que a disposição dos motores não permite controlar o robô em todas as direções.")
            print(f"Detalhe do erro: {e}")
            raise
        except Exception as e:
            print(f"ERRO DESCONHECIDO ao calcular matriz: {e}")
            raise

    def calcular_motores(self, Fx, Fy, Mz):
        '''
        Função que traduz a força desejada para aquele movimento solicitadp em comandos para os motores
        Entrada: Fx (Surge), Fy (Sway), Mz (Yaw)
        Saída: Vetor u com 4 valores entre -1 e 1 (PWM normalizado)
        '''
    
        # Cria o vetor Tau (Forças e Torque)
        tau = np.array([Fx, Fy, Mz], dtype=float)

        if not np.isfinite(tau).all():
            print(f"Alerta: Entrada inválida recebida para o Tau! Tau: {tau}")
            return np.zeros(4)
        
        try: 
            #u = B_pinv * tau
            u = self.B_pinv @ tau

            #O valor de u não posso passar de +-1 (+-100%), se acontecer reduzimos todos proporcionalmente
            # Isso garante que o robô não perca a direção do movimento
            valor_maximo = np.max(np.abs(u))

            if valor_maximo > 1.0:
                u = u / valor_maximo #Normaliza para manter a proporção
                print(f"Aviso: Comandos saturados. Escalado por {1/valor_maximo:.2f}")

            return u   
        
        except Exception as e:
                print(f"ERRO CRÍTICO no cálculo dos motores: {e}")
                return np.zeros(4) # Para os motores em caso de erro

if __name__ == "__main__":
    mixer = Controle_de_Alocação()
    print("\n--- TESTE DE FUNCIONAMENTO ---")
    
    # Teste 1: Tentar girar no sentido horário (Mz = 1)
    #res = mixer.calcular_motores(0, 0, 1)
    #print(f"Teste Giro (Mz=1): {np.round(res, 3)}")
    # Esperado: Sinais alternados [- + + -]
    
    # Teste 2: Saturação (Pedir força infinita)
    #res_sat = mixer.calcular_motores(100, 0, 0)
    #print(f"Teste Saturação (Fx=100): {np.round(res_sat, 3)}")
    # Esperado: Valores travados em 1.0 ou -1.0

