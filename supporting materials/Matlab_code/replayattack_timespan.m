%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������ڲ������������طŹ���ʱ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear,clc
%% ������ȷ���Ĺ���ʱ����ΧΪ[0-11]
attack_X=10;
xi=1/5;
x=0:1:attack_X;
for i=1:attack_X+1
    f(i)=xi*exp(-xi*x(i));%�����ܶȺ������ο�Optimal periodic watermarking schedule��
end
%plot(x,f)
max_attacktime=size([f 1-sum(f)],2);%ȷ�����Ĺ���ʱ��Ϊmax_attacktime-1

basic_length=10;%���û�������ʱ��
split=5;        %�����طŹ�����Ƭ����
attack_length_set=[];%��Ź�������Ƭ�εĹ���ʱ��
attack_start_set=[];%��Ÿ���Ƭ�εĹ�����ʼʱ��
for i=1:split
    attack_length=randsrc(1,1,[[0:max_attacktime-1];[f 1-sum(f)]]);%��ǰʱ�̵Ĺ���ʱ��
    attack_length_set=[attack_length_set attack_length+basic_length];
    attack_start=randi([50*i 50*i+25],1,1);         %������Ըı���ʼʱ��t_0��ѡ��
    attack_start_set=[attack_start_set attack_start];
end
time=300;
replay_attack=zeros(1,time);%���ڴ���طŹ���
for j=1:split
    for i=1:time
        if i>=attack_start_set(j) & i<attack_start_set(j)+attack_length_set(j)
            replay_attack(i)=1;
        end
    end
end
%plot(1:time,replay_attack,'-');


%% ������ϵͳ���й��̣�ϵͳ����ʱ������˵������
N=time; %����ʱ�䣬ʱ����������

%% ����
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

%% ��ʼ��
X=zeros(4,N);%������ʵ״̬
X(:,1)=[1;1;2;3];%��ʼλ�ƺ��ٶ�
P0=[0.0931 0.0040 0 0;0.0040 0.9809 0 0;0 0 0.1064 0.0017;0 0 0.0017 1.0759];%��ʼ���
%Z=zeros(1,N);
Z=zeros(2,N);
%Z(1)=H*X(:,1);%��ʼ�۲�ֵ
Z(:,1)=H*X(:,1);
Xkf=zeros(4,N);%����������״̬��ʼ��
Xkf(:,1)=X(:,1);
X_pre_set=zeros(4,N);%����������״̬�ĳ�ʼ��
X_pre_set(:,1)=X(:,1);
err_P=zeros(N,4);

%% ϵͳ���й���
I=eye(4); %4άϵͳ
for k=2:N
    X(:,k)=A*X(:,k-1)+B*U+W(k);         %�������䣬��״̬���̵�����
    Z(:,k)=H*X(:,k)+V(k);               %λ�ƴ�������Ŀ����й۲�
    %�������˲�
    X_pre=A*Xkf(:,k-1)+B*U;             %״̬Ԥ�� 
    X_pre_set(:,k)=X_pre;
    P_pre=A*P0*A'+Q;                    %Э����Ԥ��
    Kg=P_pre*H'*inv(H*P_pre*H'+R);      %���㿨��������
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);   %״̬����
    P0=(I-Kg*H)*P_pre;                  %�������
end

%% ������ȷ���Ĺ���ʱ����ΧΪ[0-98]
% attack_X=97;
% xi=1/25;
% x=0:1:attack_X;
% for i=1:attack_X+1
%     f(i)=xi*exp(-xi*x(i));%�����ܶȺ������ο�Optimal periodic watermarking schedule��
% end
% %plot(x,f)
% max_attacktime=size([f 1-sum(f)],2);%ȷ�����Ĺ���ʱ��Ϊmax_attacktime-1
% attack_length=randsrc(1,1,[[0:max_attacktime-1];[f 1-sum(f)]]);%��ǰʱ�̵Ĺ���ʱ��