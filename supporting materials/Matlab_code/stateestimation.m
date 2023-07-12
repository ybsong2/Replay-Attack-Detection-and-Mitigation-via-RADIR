%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����������ģ���طŹ�����ʵ�ֹ���
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc,clear
N=1000; %����ʱ�䣬ʱ����������


Q=[1,0;0,1];%������������Ϊ0����������̺��Կ�������
R=[1 0;0 1];
A=[-0.0022 -0.0030; 1.0625 0.9808];%״̬ת�ƾ���
B=[0;0];%������
U=-1;
H=[0 0.170667;0 0];%�۲����
W=sqrt(Q)*randn(2,N);%��ȻQΪ0����W=0
V=sqrt(R)*randn(2,N);

%a=[0.9394 0 0.0575 0;0 0.9508 0 0.0496;0 0 0.9406 0;0 0 0 0.9491]

%% %% �ջ��¼���������
%��ʼ��
X=zeros(2,N);%������ʵ״̬
X(:,1)=[0;0];%��ʼλ�ƺ��ٶ�
%P0=[1.3944 -0.0176;-0.0176 0.3528];
P0=[1.0001 -0.0183;-0.0183 7.0516];
%Z=zeros(1,N);
y=zeros(2,N);
%Z(1)=H*X(:,1);%��ʼ�۲�ֵ
y(:,1)=H*X(:,1);
Xkf=zeros(2,N);%����������״̬��ʼ��
Xkf(:,1)=X(:,1);
err_P=zeros(N,2);
err_P(1,1)=P0(1,1);
err_P(1,2)=P0(2,2);
I=eye(2); %��άϵͳ
for k=2:N
    X(:,k)=A*X(:,k-1)+B*U+W(k);         %�������䣬��״̬���̵�����
    Z(:,k)=H*X(:,k)+V(k);               %λ�ƴ�������Ŀ����й۲�
    %�������˲�
    X_pre=A*Xkf(:,k-1)+B*U;             %״̬Ԥ�� 
    P_pre=A*P0*A'+Q;                    %Э����Ԥ��
    Kg=P_pre*H'*inv(H*P_pre*H'+R);      %���㿨��������
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);   %״̬����
    P0=(I-Kg*H)*P_pre;                  %�������
end
t_record_start=201;
t_start=501;
t_end=600;
for k=1:N
    if k>=t_start&k<=t_end
        X_attack(:,k)=Xkf(:,t_record_start+k-t_start);
    else
        X_attack(:,k)=Xkf(:,k);
    end
    coverror_normal(k)=(X(:,k)-Xkf(:,k))'*(X(:,k)-Xkf(:,k));
    coverror_attck(k)=(X(:,k)-X_attack(:,k))'*(X(:,k)-X_attack(:,k));
end
plot(1:N,X(1,:),'-c',1:N,Xkf(1,:),'b--');
figure
plot(1:N,X_attack(1,:),'-c',1:N,Xkf(1,:),'b--');
figure
plot(1:N,coverror_normal,'-r',1:N,coverror_attck,'b--');
figure 
plot(1:N,coverror_normal-coverror_attck)