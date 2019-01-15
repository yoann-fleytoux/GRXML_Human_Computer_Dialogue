%% Commande articulaire d'un robot manipulateur de type RP
%
% TP r�alis� par Benjamin CHAMAND
%
% Cette manipulation concerne l'�tude simul�e, sous Matlab, de diverses
% strat�gies pour la commande articulaire en position d'un bras
% manipulateur �l�mentaire de type RP.

%% Prise en main de l'outil de simulation et calculs pr�liminaires
% *Question 1 :* �volution temporelle des profils de consigne.
%
% En simulant le bloc "profils_de_consigne_axes_1_et_2" avec 2 
% oscilloscopes � la sortie du bloc on voit le profil de position, de
% vitesse et d'acc�l�ration de q1 et q2.

%%
% 
% <<../img/profils_de_consignes.png>>
% 

%%
% Scope 1: montre l'�volution temporelle de la position, vitesse et 
% acc�l�ration de q1
%
% <<../img/q1.png>>
% 

%%
% Scope 2: montre l'�volution temporelle de la position, vitesse et 
% acc�l�ration de q2
% 
% <<../img/q2.png>>
% 

%%
% *Question 2 :* Mise en place des constantes 

% Rapport du gain en vitesse du moteur et de la r�sistance de l'induit
Km_R = 0.3;

% Coefficient de frottement efficace de l'ensemble moteur-r�ducteur
Beff = 1/80;

% Inertie ensemble moteur-r�ducteur
Jm = 1/100;

% Rapports de r�duction (3 cas diff�rents)
R = [1/200 1/30 1];
% On consid�re un cas du rapport de r�duction parmi les 3 pr�c�dents
r = R(1);

% masse m = 15kg
m = 15;

% Energie efficace
Jeff = Jm + r^2*m;

% Fonction de transfert de la partie m�canique du robot
G = tf(1,[Jeff Beff 0]);


%% Commande en vitesse de type PD
% *Exercice 3 :* On doit �tablir l'expression analytique des coefficients
% $K$ et $K_D$ de la loi de commande proportionnelle d�riv�e.
%
% $$
% \epsilon_{vit} = \frac{(B_{eff} + K_d\cdot \frac{K_m}{R})\dot{\theta_1} + r\cdot
% d0}{K\frac{K_m}{R}}
% $$
%
% On a $d_i(t) \equiv 0$, par suite :
%
% $$
% \epsilon_{vit} = \frac{(B_{eff} + K_d\cdot \frac{K_m}{R})\dot{\theta_1}}{K\frac{K_m}{R}}
% $$
%
% De plus, avec $\zeta = 1$, on a :
%
% $$
% w_n = \sqrt{\frac{K\cdot \frac{K_m}{R}}{J}}
% $$
%
% $$
% 2\cdot \zeta \cdot w_n = \frac{B_{eff} + K_d\cdot \frac{K_m}{R}}{J}
% \Rightarrow 2 \cdot w_n = \frac{B_{eff} + K_d\cdot \frac{K_m}{R}}{J}
% $$
%
% En �galisant les 2 derni�res �quations, on trouve :
%
% $$
% K = \frac{B_{eff}+ k_d\cdot \frac{K_m}{R}}{4\cdot J_{eff}\cdot \frac{K_m}{R}}
% $$


%%
% *Question 4 :* 

%%
% *(a) :* calcul des valeurs num�riques des coefficients $K$ et $K_d$ de 
% telle sorte que l'erreur en vitesse sur la consigne rampe = 1/2.
%
% Avec toutes les �quations pr�c�dentes, on a : 
% 
% $$
% K = \frac{2\cdot (B_{eff} + K_d\cdot \frac{K_m}{R}}{K\cdot \frac{K_m}{R}}
% = \frac{B_{eff}+ k_d\cdot \frac{K_m}{R}}{4\cdot J_{eff}\cdot \frac{K_m}{R}}
% $$
%
% $$
% \Rightarrow K_d = \frac{8\cdot J_{eff} - B_{eff}}{\frac{K_m}{R}}
% $$
%
% On trouve alors $K$ en utilisant $K_d$ :
%
% $$
% K = \frac{2\cdot (B_{eff} + 8\cdot J_{eff} - B)}{\frac{K_m}{R}} =
% \frac{16 \cdot J_{eff}}{\frac{K_m}{R}}
% $$

K = (16*Jeff) / Km_R;
Kd = (8*Jeff - Beff) / Km_R;

%%
% *(b) :* Lieux de transfert de la boucle ouverte :
d = 1;
Bo = series((Km_R-r*d),G);
Bf = feedback(K*Bo,tf([Kd 0],1));
margin(Bo);
title('Marge de phase et marge de gain');
hold on;
margin(Bf);
legend('Bo', 'Bf');
hold off;

%%
% Avant correction, la marge de phase est faible proche de -180�
% Apres correction, la marge de phase est plus grande au point critique.

%%
% *(c) :* Simulink
% En l'absence de pertubation, l'erreur de position est nulle et l'erreur
% de vitesse est bien de 1/2.
% 
% Avec la perturbation, le syst�me n'est pas capable de corriger la
% perturbation et cette derni�re s'ajouter � notre erreur de position
% ainsi que de vitesse.

%%
% Mod�le lin�aire : 
%
% <<../img/4_lineaire.png>>
% 

%%
% Mod�le non lin�aire : 
%
% <<../img/4_nonlineaire.png>>
% 

%%
% *Question 5 :* 
%
% On a un mod�le lin�aire valable pour un rapport de r�duction r �lev�.
% S'il n'y a pas de de perturbations et pas de gravit�, le syst�me est
% correctement asservi
%
% Le mod�le met plus de temps � se stabiliser dans le cas non lineaire.


%% Commande en vitesse de type PID
% On se place dans l'hypoth�se o� r=1/200

%%
% *Question 6 :* calcul de $T_I$ :

Ti = 20000;
BFPID = Bf * tf([Ti 1], [Ti 0]);

%%
% Il faut retoucher le Ti pour l'axe 2 afin que la correction soit
% satisfaisante sur q2
Ti2 = 12;

%%
% Dans la partie simulink nous remarquons que les sorties du syst�me n'ont
% pas d'erreur en regime permanent alors qu'elles ont un faible retard ainsi
% qu'un d�passement selon la valeur de Ti.

%% Retour sur la mod�lisation
% *Question 7 :* Expression de Jeffi(q) et rdi(q) fonctions des coordonn�es
% articulaires du robot.
%
% Actionneur #k
%
% $$
% J_m\cdot \ddot \theta_m + B_{eff}\cdot \dot \theta_m = \frac{K_m}{R}\cdot
% v_k - r\cdot d_k
% $$
%
% $$
% \begin{array}{rl}
% r\cdot d_k & = r\cdot (D\cdot \ddot q+B+G)\\
%  & = r\cdot \left (\begin{array}{cc} m\cdot q_2^2 & 0\\ 0 & m\end{array}\right)
%   \cdot \left (\begin{array}{c}\ddot q_1\\ \ddot q_2\end{array}\right )
%  + r\cdot \left (\begin{array}{c}2\cdot m\cdot q_2\cdot \dot q_1\cdot
%  \dot q_2\\ -m\cdot q_2\cdot \dot q_1^2\end{array}\right )
%  - r\left (\begin{array}{c}m\cdot q_2\cdot g_y\cdot \cos(q_1)\\ m\cdot
%  g_y\cdot \sin(q_1)\end{array}\right)\\
%  & = r\cdot D\cdot \left (\begin{array}{c}\ddot q_1\\ \ddot
%  q_2\end{array}\right ) + r\cdot \left (\begin{array}{c} d_1\\
%  d_2\end{array}\right)
% \end{array}
% $$
%
% Avec :
%
% $$
% \begin{array}{rl}
% d_k & = B+G\\
%  & = \left (\begin{array}{c}2\cdot m\cdot q_2\cdot \dot q_1\cdot
%  \dot q_2\\ -m\cdot q_2\cdot \dot q_1^2\end{array}\right ) - 
%  \left (\begin{array}{c}m\cdot q_2\cdot g_y\cdot \cos(q_1)\\ m\cdot
%  g_y\cdot \sin(q_1)\end{array}\right)\\
%  & = \left (\begin{array}{c} 2\cdot m\cdot q_2\cdot \dot q_1\cdot
%  \dot q_2 - m\cdot q_2\cdot g_y\cdot \cos(q_1)\\ -m\cdot q_2\cdot \dot
%  q_1^2 - m\cdot g_y\cdot \sin(q_1)\end{array}\right)
% \end{array}
% $$
%
% Avec toutes les �critures pr�c�dentes, on peut �crire :
%
% $$
% (J_m + r^2\cdot D)\cdot \ddot \theta_m + B_{eff}\cdot \dot \theta_m
% = \frac{K_m}{R}\cdot v_k - r\cdot (B+G)
% $$
%
% On peut donc approxim� une inertie efficace par :
%
% $$
% J_{eff} = valeur\ pire\ cas\ de\ (J_m + r^2\cdot D)
% $$


%%
% *Question 8 :* Pour chaque axe $i$, on va calculer � l'aide de Matlab les
% inerties efficaces extr�mes.

q1min = 0;
q1min = pi/2;
q2min = 0.5;
q2max = 1;

Dmin = [m*q2min^2 0 ; 0 m];
Dmax = [m*q2max^2 0 ; 0 m];
d11min = Dmin(1,1);
d22min = Dmin(2,2);
d11max = Dmax(1,1);
d22max = Dmax(2,2);

Jeff1min =  Jm + r*d11min
Jeff1max =  Jm + r*d11max
Jeff2min =  Jm + r*d22min
Jeff2max =  Jm + r*d22max

%%
% Plus un syst�me a de l'inertie, plus il est dur � contr�ler, donc on
% prend les valeurs dont l'inertie est la plus faible, c'est � dire :
% Jeff1min et Jeff2min

%% Commande non lin�aire centralis�e par anticipation
%
% Impl�mentation du feedforward :
%
% <<../img/feedforward.png>>
%
%%
% Code de l'impl�mentation sur Matlab
%
% <include>sf_loi_avant_RP.m</include>
%
%%
% R�alisation du montage sous Simulink
% 
% <<../img/5.png>>
%
%%
% On obtient les r�ponses suivantes pour l'axe 1 et 2 pour r=1/200
% 
% <<../img/q5.png>>
% 
%%
% A l'aide de plusieurs simulations, on remarque que plus on augmente la
% valeur de r, plus la valeur de l'axe 2 devient aberrante. Pour un
% r=1/200, la r�ponse est tr�s bonne, la valeur voulue pour l'axe 1 et 2
% est atteinte rapidemment.


%% Commande non lin�aire centralis�e par d�couplage
% On souhaite maintenant mettre en place une commande en boucle ferm�e
% lin�arisante d�couplante.

%%
% Code de l'impl�mentation sur Matlab
%
% <include>sf_retour_linearisant_RP.m</include>

%%
% Nouvelle valeur de K et Kd pour la commande PD
Kd = 3;
K = (Kd^2)/4;

%%
% Montage du syst�me via Simulink
%
% <<../img/decouplage.png>>
%

%%
% R�sultat via un montage Simulink pour r=1/200
%
% <<../img/scope_decouplage.png>>
%

%%
% On peut remarquer que le syst�me a juste un l�ger retard sur le temps de
% mont�e mais ex�cute bien la commande voulue.

