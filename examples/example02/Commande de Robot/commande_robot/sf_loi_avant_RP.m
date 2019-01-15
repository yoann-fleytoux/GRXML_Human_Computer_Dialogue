function [sys,x0,str,ts] = sf_loi_avant_RP(t,x,u,flag)
% NE PAS MODIFIER CI-DESSOUS ; CF. SEULEMENT mdlOutputs PLUS BAS
%
switch flag,
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;
  case 3,
    sys = mdlOutputs(t,x,u);
  case {1,2,4,9}
    sys = [];
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes
% NE PAS MODIFIER CI-DESSOUS ; CF. SEULEMENT mdlOutputs PLUS BAS
%
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;               % [d1;d2]
sizes.NumInputs      = 6;               % [q1;q2;dq1;dq2;ddq1;ddq2]
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [-1 0];                           % p�riode h�rit�e du bloc p�re
%=============================================================================
function sys = mdlOutputs(t,x,u)
%
q1 = u(1); q2 = u(2); dq1 = u(3); dq2 = u(4); ddq1 = u(5); ddq2 = u(6);
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% � COMPL�TER � PARTIR D'ICI %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sys DOIT �TRE INTIALIS� AVEC LE CONTENU DU VECTEUR DE SORTIE DU BLOC
% e.g.
Km_R = 0.3;
Beff = 1/80;
Jm = 1/100;
R = [1/200 1/30 1];
r = R(1);
m = 15;
Jeff = Jm+r^2*m;
gy = -9.81;
d1 = 2*m*q2*dq1*dq2-m*q2*gy*cos(q1);
d2 = -m*q2*dq1*dq1-m*gy*sin(q1);
s1 = ddq1/r * Jeff/Km_R + dq1/r * Beff/Km_R + r*d1/Km_R;
s2 = ddq2/r * Jeff/Km_R + dq2/r * Beff/Km_R + r*d2/Km_R;
sys = [s1;s2];

%=============================================================================
