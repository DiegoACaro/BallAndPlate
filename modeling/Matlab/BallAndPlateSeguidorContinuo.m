
%Supongamos valores para las constantes:
const = 0.027 / (0.027 + 4.32e-6 / 0.02^2);
g = 9.81;

a = [ 0  1   0  0;
      0  0  const*g 0;
      0  0   0  1;
      0  0   0  0 ];

b = [ 0;0;0;1];



c = [ 1 0 0 0];


d = 0;


%a=[0,1;-((l*Mm*g)/IT),-((l*M*g)/IT)];
%b=[0;l/IT]
%c=[1,0] %Angulo
%d=0


pd=[-2+4.5826*i,-2-4.5826*i];



aa=[a,zeros(4,1);-c,0]
ba=[b;0]

pda=[-40, -60, -70, -80];
kt=place(aa,ba,pda)
kp=kt(1:1,1:2)
ki=kt(1:1,3:3)



po=[-10,-11]
h=place(a',c',po)
h=h'


[numu,denu]=ss2tf(a-h*c,b,kp,0)
[numy,deny]=ss2tf(a-h*c,h,kp,0)


c2g3_simu_25092024_seguidorconobs