import numpy as np

class Controle_de_Alocação:
    """
    Implementação da Alocação de Controle Não-Restrita.
    Referência: Fossen, T. I. (2011). Handbook of Marine Craft Hydrodynamics and Motion Control.
    Capítulo 12.3.2
    """
    def __init__(self):
        print("Iniciando o sistema de alocação do controle (fase 1)")

        #1. Definição da Geometria
        '''1 = Frente-Esquerda , 2 = Frente-Direita , 3 = Traseira-Esquerda , 4 = Traseira-Direita

        Com todos os propulsores simetricos à 45º da horizontal ou vertical têm-se:
        Prop1 (Frente-Esq): Jato sai p/ Frente-Dir -> Força p/ Trás-Esq (alpha = 225º)
        Prop2 (Frente-Dir): Jato sai p/ Frente-Esq -> Força p/ Trás-Dir (alpha = 135º)
        Prop3 (Trás-Esq): Jato sai p/ Trás-Dir     -> Força p/ Frente-Esq (alpha = 315º)
        Prop4 (Trás-Dir): ato sai p/ Trás-Esq      -> Força p/ Frente-Dir (alpha = 45º)'''
        # alpha: Ângulos de azimute dos propulsores em radianos
        # Sentido: Horário positivo (NED)
        self.alpha_deg = np.array([225, 135, 315, 45])
        self.alpha = np.radians(self.alpha_deg)

        '''Coordenadas dos propulsores em relação ao centro de massa obtido no Fusion360
        # Formato: [x (metros), y (metros)]
        # x+ = Frente, x- = Trás
        # y+ = Direita, y- = Esquerda'''
        # r_thrusters: Posição (braços de alavanca) de cada propulsor [lx, ly]
        # lx (Surge): Positivo para frente
        # ly (Sway): Positivo para direita
        self.r_thrusters = np.array([
            [0.09792375 , -0.07528369], #Prop1 (Frente-Esq)
            [ 0.0979239,  0.07670449], #Prop2 (Frente-Dir)
            [-0.0815487, -0.09728376], #Prop3 (Trás-Esq)
            [-0.08154855,  0.09870441]  #Prop4 (Trás-Dir)
        ])

        num_motores_ang = len(self.alpha)
        num_motores_pos = len(self.r_thrusters)

        # Verificação de Integridade Geométrica
        if num_motores_ang != 4 or num_motores_pos != 4: 
            erro_mensagem = f"Erro: Geometria inserida é consistente! Ângulos: {num_motores_ang}, Coords: {num_motores_pos}."
            print(erro_mensagem)
            raise ValueError(erro_mensagem)
        
        if num_motores_ang != num_motores_pos:
            erro_mensagem = f"Erro: Número de ângulos diferente do número de coordenadas."
            print(erro_mensagem)
            raise ValueError(erro_mensagem)
        
        print("f-> Geometria Funcional: {num_motores_pos} propulsores configurados.")

        #2 Calculo da Matriz de configuração (T)
        #eq 12.244 em Fossen: tau = T * f
        try: 
            #Extração de vetores para cálculo vetorial
            lx = self.r_thrusters[:, 0]
            ly = self.r_thrusters[:, 1]

            #Cálculo do Braço de Momento (N) para Yaw
            '''N = lx*Fy - ly*Fx (prod. vetorial)
            usa-se sin/cos dos ângulos para projetar a força
            N = lx * sin(alpha) - ly * cos(alpha)'''
            # Nota: Fossen define a contribuição de momento cruzado
            moment_arms = lx * np.sin(self.alpha) - ly * np.cos(self.alpha)

            #Matriz de Configuração T (3DOF) 3x4
            '''
            Linha 0: Força em Surge (x)-> cos(alpha)
            Linha 1: Força em Sway (y) -> sin(alpha)
            Linha 2: Momento em Yaw (N) -> moment_arms
            '''
            self.T = np.array([
                np.cos(self.alpha), #Contribuição em X
                np.sin(self.alpha), #Contribuição em Y
                moment_arms        #Contribuição no Giro
            ])

            #3 Inversão Matricial
            #Cálculo da matriz Pseudo-Inversa de Moore-Penrose
            #eq 12.253: Solução de Mínimos Quadrados 
            #f = T_pinv * tau
            self.T_pinv = np.linalg.pinv(self.T, rcond = 1e-5) #rcond=1e-5 para ignorar valores muito pequenos (erro numérico)

            print("-> Matriz de Alocação calculada e invertida")
            # Debug: Mostrar a matriz para conferência visual
            print("Matriz B:\n", np.round(self.B, 2))

            cond_numero = np.linalg.cond(self.B)
            print(f"Condition Number: {cond_numero:.2f}")

            if cond_numero > 20.0:
                print("[AVISO] Geometria ineficiente! Os motores podem estar 'brigando'.")
                print("Verifique se os ângulos alpha não estão muito alinhados.")
            else:
                print("[OK] Geometria bem condicionada.")

        except np.linalg.LinAlgError as e:
            print("Erro matemático: A matriz de configuração é singular")
            print("Isso significa que a disposição dos motores não permite controlar o robô em todas as direções.")
            print(f"Detalhe do erro: {e}")
            raise
        except Exception as e:
            print(f"ERRO DESCONHECIDO ao calcular matriz: {e}")
            raise

    def calcular_motores(self, X, Y, N):
        '''
        Função que traduz a força desejada para aquele movimento solicitadp em comandos para os motores
        Converte as forças generalizadas (tau) em comandos de atuadores (u)
        Entrada:
        X: Força desejada em Surge (N)
        Y: Força desejada em Sway (N)
        N: Momento desejado em Yaw (N*m)
        Saída: Vetor u (np.array): Vetor de comando normalizado [-1, 1] para os 4 motores.
        '''
    
        # 1. Cria o vetor Tau (Forças generalizadas)
        #Eq. 12.244: tau = [X, Y, N]^T
        tau = np.array([X, Y, N], dtype=float)

        if not np.isfinite(tau).all():
            print(f"Alerta: Entrada inválida em tau: {tau}. Retornando zero.")
            return np.zeros(4)
        
        try: 
            # 2. Alocação de Controle 
            # Eq. 12.253: u = T_pinv * tau
            f = self.T_pinv @ tau

            #Nota: f é a força ideal requerida em cada motor
            #Desse modo, como o a pseudo-inversa já retorna o mínimo quadrado, f já é a melhor solução e deve ser tratado como o sinal de controle u
            u = f

            #Excluir resultados pequenos (motor muito fraco)
            u[np.abs(u) < 0.05] = 0.0 

            #O valor de u não posso passar de +-1 (+-100%), se acontecer reduzimos todos proporcionalmente
            # Isso garante que o robô não perca a direção do movimento
            valor_maximo = np.max(np.abs(u))

            if valor_maximo > 1.0:
                u = u / valor_maximo #Normaliza para manter a proporção
                print(f"Aviso: Comandos saturados. Escalado por {1/valor_maximo:.2f}")
                if valor_maximo > 1.1:
                    print(f"aturação muito grande: {valor_maximo*100:.0f}% (Escalado)")

            return u   
        
        except Exception as e:
                print(f"ERRO de cálculo na alocação dos motores: {e}")
                return np.zeros(4) # Para os motores em caso de erro

if __name__ == "__main__":
    Control_Alloc = Controle_de_Alocação()

    print("\n Testes simples ")
    
    # Caso 1: Força Pura em Surge (X=1, Y=0, N=0)
    print("Teste 1: Surge Puro (X=1.0)")
    u_surge = Control_Alloc.calcular_atuadores(1.0, 0.0, 0.0)
    print(f"u = {np.round(u_surge, 3)}")
    
   # Caso 2: Momento Puro em Yaw (X=0, Y=0, N=1)
    print("Teste 2: Yaw Puro (N=1)")
    u_yaw = Control_Alloc.calcular_atuadores(0.0, 0.0, 1.0)
    print(f"   u = {np.round(u_yaw, 3)}")