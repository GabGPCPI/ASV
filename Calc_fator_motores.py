import numpy as np

class AlocaçãoDeControle:
    def __init__(self):
        #1 = Frente-Esquerda , 2 = Frente-Direita , 3 = Traseira-Esquerda , 4 = Traseira-Direita

        #Com todos os propulsores simetricos à 45º da horizontal ou vertical têm-se:
        #Prop1 (Frente-Esq): Jato sai p/ Frente-Dir -> Força p/ Trás-Esq (alpha = 225º)
        #Prop2 (Frente-Dir): Jato sai p/ Frente-Esq -> Força p/ Trás-Dir (alpha = 135º)
        #Prop3 (Trás-Esq): Jato sai p/ Trás-Dir     -> Força p/ Frente-Esq (alpha = 315º)
        #Prop4 (Trás-Dir): ato sai p/ Trás-Esq      -> Força p/ Frente-Dir (alpha = 45º)

        self.alpha = np.radians([225, 135, 315, 45])

        #COORDENADAS DOS PROPULSORES em relação ao centro de massa obtido no Fusion360
        # Formato: [x (metros), y (metros)]
        # x+ = Frente, x- = Trás
        # y+ = Direita, y- = Esquerda

        self.props_coords = np.array([
            [0.09792375 , -0.07528369], #Prop1 (Frente-Esq)
            [ 0.0979239,  0.07670449], #Prop2 (Frente-Dir)
            [-0.0815487, -0.09728376], #Prop3 (Trás-Esq)
            [-0.08154855,  0.09870441]  #Prop4 (Trás-Dir)
        ])
        prop_x = self.props_coords[:, 0] #Propulsores em x
        prop_y = self.props_coords[:, 1] #Propulsores em y

        #CÁLCULO DO MOMENTO EM TORNO DE Z (Mz)
        #Mz = x*Fy - y*Fz = *sin(alpha) - y*cos(alpha)
        braço_momento_z = prop_x * np.sin(self.alpha) - prop_y * np.cos(self.alpha)
        print("Braços de Momento em Z calculados:", np.round(braço_momento_z, 3))

        #Definindo a matriz B
        self.B = np.array([
            np.cos(self.alpha), #Fx
            np.sin(self.alpha), #Fy
            braço_momento_z #Mz
        ])

        self.B_pinv = np.linalg.pinv(self.B)

    def calcular_motores(self, Fx, Fy, Mz):
        tau = np.array
        u = self.B_pinv @ tau
        max_val = np.max(np.abs(u))
        if max_val > 1:
            u = u / max_val
        return u
    
if __name__ == "__main__":
    mixer = AlocaçãoDeControle()
    print("\nTeste Giro (Mz=1):", np.round(mixer.calcular_motores(0,0,1), 3))
