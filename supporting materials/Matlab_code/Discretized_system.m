%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������ڽ�����ϵͳ��A��B��������ɢ�����õ���Ӧ��G��H��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��ɢ�����ϵͳ����ΪG���������ΪH������CD��ɢ��ǰ�󱣳ֲ��䡣
dt=0.01;   %�������
% A=[0,1;-2,-3];
% B=[0;1];
A=[0 1 0 0;0 -0.5 0 0;0 0 0 1;0 0 0 -0.5];
B=[0 0;0.5 0;0 0;0 0.5];

%% ��ʽ�任��������
syms s t ls;         %��״̬ת�ƾ���  ������ʽ�任
    I=eye(size(A));
    Ls=inv(s*I-A);
    STM=ilaplace(Ls,s,t);%״̬ת�ƾ���
    G=double(subs(STM,t,dt));%���ź������
syms T
    HLs=int(STM,t,0,T);
    H=HLs*B;
    H=real(double(subs(H,T,dt)));%���ź������
%[Ak,Bk]=c2d(A,B,dt); 











% G=exp(A*t);
% syms k
% pha=symsum(A.^k*t.^k/factorial(k),k,0,inf);