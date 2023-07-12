%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������ڼ���4άϵͳ�ĳ�ֵ\bar_P
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear;

N=300; %����ʱ�䣬ʱ����������
%N=10000;
%% ����
%q=[0.003 1.0000 -0.005 -2.150]';
%q=[0.01 0.01 0.01 0.01];
duijiao2=[0.131 0.123];
Q=diag([0.228 0.0102 0.683 0.0109]);%������������Ϊ0����������̺��Կ�������
R=diag(duijiao2); %�۲���������
W=sqrt(Q)*randn(4,N);%��ȻQΪ0����W=0
V=sqrt(R)*randn(2,N);%��������V(k)

%% ϵ������
A=[1.0000,0.0100,0.0000,0.0000;0.0000,0.9950,0.0000,0.0000;0.0000,0.0000,1.00000,0.0100;0.0000,0.0000,0.0000,0.9950];%״̬ת�ƾ���
B=[2.49583853646267e-05,0;0.00498752080731769,0;0,2.49583853646267e-05;0,0.00498752080731769];                       %�������
U=[0;0];
H=[1 0 0 0;0 0 1 0];%�۲����
state_num=4; 
%% ������ⷴ������L
setlmis([]);
P=lmivar(1,[state_num,1]);
K=lmivar(2,[2,state_num]);

lmiterm([-1 1 1 P],1,1);

lmiterm([2 1 1 P],-1,1);
lmiterm([2 2 1 K],B,1);
lmiterm([2 2 1 P],A,1);
lmiterm([2 2 2 P],-1,1);



lmisys=getlmis;
[tmin,xfeas]=feasp(lmisys);
K=dec2mat(lmisys,xfeas,K);
P=dec2mat(lmisys,xfeas,P);

L=K*inv(P);%��������L
%[L,l1,l2]=lqr(A,B,eye(4),eye(2));
%��ʼ��
X=zeros(4,N);%������ʵ״̬
X(:,1)=[1;1;2;3];%��ʼλ�ƺ��ٶ�
P0=[0.0931 0.0040 0 0;0.0040 0.9809 0 0;0 0 0.1064 0.0017;0 0 0.0017 1.0759];%��ʼ���
%Z=zeros(1,N);
Z=zeros(2,N);
%Z(1)=H*X(:,1);%��ʼ�۲�ֵ
Z(:,1)=H*X(:,1);
Xkf=zeros(4,N);%����������״̬��ʼ��
Xkf(:,1)=X(:,1);
err_P=zeros(N,4);
err_P(1,1)=P0(1,1);
err_P(1,2)=P0(2,2);

%% �������
P0=[0.0931 0.0040 0 0;0.0040 0.9809 0 0;0 0 0.1064 0.0017;0 0 0.0017 1.0759];
Kg=(A*P0*A'+Q)*H'*inv(H*(A*P0*A'+Q)*H'+R);
cvx_begin
    variable sum_star(2,2) diagonal;
    variable sum_H1(2,2);
    variable mathcal_L(4,4);
    variable tau;
    minimize(trace(sum_star));
    subject to 
        sum_star>0;
        tau<=0.1;
        sum_H1-(H*P0*H'+R)-2*sum_star-H*mathcal_L*H'==0;
        mathcal_L-(A+B*L)*(eye(4)-Kg*H)*mathcal_L*((A+B*L)*(eye(4)-Kg*H))'-(A+B*L)*Kg*sum_star*((A+B*L)*Kg)'==0;
 cvx_end
% sum_star=diag([10,10]);
y_star=sqrt(sum_star)*randn(2,N);%��������V(k)
g(1)=0;
I=eye(4); %��άϵͳ
for k=2:N
    X(:,k)=A*X(:,k-1)+B*U+W(k);         %�������䣬��״̬���̵�����
    Z(:,k)=H*X(:,k)+V(k)+y_star(k);               %λ�ƴ�������Ŀ����й۲�
    %�������˲�
    X_pre=A*Xkf(:,k-1)+B*U;             %״̬Ԥ�� 
    P_pre=A*P0*A'+Q;                    %Э����Ԥ��
    %Kg=P_pre*H'*inv(H*P_pre*H'+R);      %���㿨��������
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);   %״̬����
    P0=(I-Kg*H)*P_pre;                  %�������
    U=L*Xkf(:,k);
    g(k)=(Z(k)-H*X_pre)'*inv(sum_H1)*(Z(k)-H*X_pre);
%     %������ֵ
%     err_P(k,1)=P0(1,1);
%     err_P(k,2)=P0(2,2);
end
figure
plot(1:N,g);
for k=2:N
    X(:,k)=A*X(:,k-1)+B*U+W(k);         %�������䣬��״̬���̵�����
    Z(:,k)=H*X(:,k)+V(k)+y_star(k);               %λ�ƴ�������Ŀ����й۲�
    %�������˲�
    if k>=72&&k<=82
        Z(:,k)=Z(:,k-50);
    elseif k>=105&&k<=117
        Z(:,k)=Z(:,k-20);
    elseif k>=162&&k<=175
        Z(:,k)=Z(:,k-40);
    elseif k>=224&&k<=234
        Z(:,k)=Z(:,k-40);
    elseif k>=250&&k<=257
        Z(:,k)=Z(:,k-15);
    end
    X_pre=A*Xkf(:,k-1)+B*U;             %״̬Ԥ�� 
    P_pre=A*P0*A'+Q;                    %Э����Ԥ��
    %Kg=P_pre*H'*inv(H*P_pre*H'+R);      %���㿨��������
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);   %״̬����
    P0=(I-Kg*H)*P_pre;                  %�������
    U=L*Xkf(:,k);
    g(k)=(Z(k)-H*X_pre)'*inv(sum_H1)*(Z(k)-H*X_pre);
%     %������ֵ
%     err_P(k,1)=P0(1,1);
%     err_P(k,2)=P0(2,2);
end
figure
plot(1:N,g);
