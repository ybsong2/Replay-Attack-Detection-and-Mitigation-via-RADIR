%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序用于将连续系统的A和B，进行离散化，得到相应的G和H。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%离散化后的系统矩阵为G，输入矩阵为H。其它CD离散化前后保持不变。
dt=0.01;   %采样间隔
% A=[0,1;-2,-3];
% B=[0;1];
A=[0 1 0 0;0 -0.5 0 0;0 0 0 1;0 0 0 -0.5];
B=[0 0;0.5 0;0 0;0 0.5];

%% 拉式变换法求解过程
syms s t ls;         %求状态转移矩阵  利用拉式变换
    I=eye(size(A));
    Ls=inv(s*I-A);
    STM=ilaplace(Ls,s,t);%状态转移矩阵
    G=double(subs(STM,t,dt));%符号函数求解
syms T
    HLs=int(STM,t,0,T);
    H=HLs*B;
    H=real(double(subs(H,T,dt)));%符号函数求解
%[Ak,Bk]=c2d(A,B,dt); 











% G=exp(A*t);
% syms k
% pha=symsum(A.^k*t.^k/factorial(k),k,0,inf);