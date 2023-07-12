%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������ڲ������������طŹ���ʱ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear,clc
%% ������ȷ���Ĺ���ʱ����ΧΪ[0-11]


max_attacktime=12;
f=[0.2000 0.1637 0.1341 0.1098 0.0899 0.0736 0.0602 0.0493 0.0404 0.0331 0.0271 0.0188];
extra_length=[0:11];

basic_length=10;%���û�������ʱ��
split=10;        %�����طŹ�����Ƭ����
attack_length_set=[];%��Ź�������Ƭ�εĹ���ʱ��
attack_start_set=[];%��Ÿ���Ƭ�εĹ�����ʼʱ��
for i=1:split
    attack_length=randsrc(1,1,[extra_length;f]);%��ǰʱ�̵Ĺ���ʱ��
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
X(:,1)=[3;1;3;1];%��ʼλ�ƺ��ٶ�
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



time=300;%���д���
p=10;%�������ڳ���Ϊ10
period_policy_number=8;%����8����ͬ�ĵ��Ȳ���
encryption_time_pro=[0.01 0.04 0.15 0.3 0.3 0.15 0.04 0.01];%����������2-9�μ��ܵĸ���
policy=cell(1,period_policy_number);
%%%���ż��ܵ��Ȳ���
policy{1}=[0 0 0 0 1 0 0 0 0 1];
policy{2}=[0 0 1 0 0 0 1 0 0 1];
policy{3}=[0 1 0 0 1 0 1 0 0 1];
policy{4}=[0 1 0 1 0 1 0 1 0 1];
policy{5}=[0 1 0 1 1 0 1 0 1 1];
policy{6}=[0 1 1 0 1 1 1 0 1 1];
policy{7}=[0 1 1 1 1 0 1 1 1 1];
policy{8}=[0 1 1 1 1 1 1 1 1 1];
%%%���м��ܵ��Ȳ���
policy_center{1}=[0 0 0 0 0 0 0 0 1 1];
policy_center{2}=[0 0 0 0 0 0 0 1 1 1];
policy_center{3}=[0 0 0 0 0 0 1 1 1 1];
policy_center{4}=[0 0 0 0 0 1 1 1 1 1];
policy_center{5}=[0 0 0 0 1 1 1 1 1 1];
policy_center{6}=[0 0 0 1 1 1 1 1 1 1];
policy_center{7}=[0 0 1 1 1 1 1 1 1 1];
policy_center{8}=[0 1 1 1 1 1 1 1 1 1];
for k=1:time
    %%%%ȷ�����ż��ܵ��Ȳ���
    if mod(k-1,p)==0
       encryption_time=randsrc(1,1,[[2:9];encryption_time_pro]);%��ǰʱ�̵ļ��ܴ���
       gamma(k:k+p-1)=policy{encryption_time-1};
       gamma_center(k:k+p-1)=policy_center{encryption_time-1};
       %%%���������ܵ��Ȳ���
       random_set=randperm(p);
       for i=1:p
           if random_set(i)<=encryption_time
               random_01_set(i)=1;
           else
               random_01_set(i)=0;
           end
       end
       gamma_random(k:k+p-1)=random_01_set;%������Լ���
    end
    %%%%Ϊ���ܵ����ñ�ǩ
    if gamma(k)==1
        label(k)=k;
        gamma_point(k)=1;
    else 
        label(k)=0;
        gamma_point(k)=NaN;
    end
    %%%%����������ʱ�����ܵ�ı�ǩ����������
    if replay_attack(k)==1
        if gamma(k)==1
            label(k)=10000;%���ýϴ�ı�ǩ��10000�����Ը���
        end
    end    
end
%% �طŹ����ָ��㷨��״̬���¹���
attack_exist=0;
tau=0;
attack_inter_set=zeros(1,time);
P_error_matrix=cell(1,time);
for k=1:time
    if gamma(k)==1
        if label(k)==k
            hat_X(:,k)=Xkf(:,k);
            P_error_matrix{k}=P0;
            tau=k;%���ڴ洢��һ�����ܵ�����Ӧ��ʱ��ֵ
            attack_exist=0;%��ʾ��ǰʱ��û�й���
            attack_inter_set(k)=attack_exist;
        else
            hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%��ʾ��ǰ
            attack_inter_set(k)=attack_exist;
        end
    else
        if attack_exist==0
            hat_X(:,k)=Xkf(:,k);
            P_error_matrix{k}=P0;
        else
            hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
        end
    end        
end
%% ģ���طŹ���
replay_attack_start=10;
X_attack=Xkf;%���ڴ���طŹ������״ֵ̬
for j=1:split
    for i=1:time
        if i>=attack_start_set(j) & i<attack_start_set(j)+attack_length_set(j)
            X_attack(:,i)=Xkf(:,replay_attack_start+i-attack_start_set(j));  %ֻ�Ѵ��ڹ������ֵ����ݽ�����Ӧ���滻
%         else
%             X_attack(:,i)=Xkf(:,i);
        end
    end
end



subplot(2,2,1)        
plot(1:time,Xkf(1,:),'-',1:time,hat_X(1,:),'-.',1:time,X_attack(1,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$x$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%�����������С
subplot(2,2,2)        
plot(1:time,Xkf(2,:),'-',1:time,hat_X(2,:),'-.',1:time,X_attack(2,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$v$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%�����������С
subplot(2,2,3)        
plot(1:time,Xkf(3,:),'-',1:time,hat_X(3,:),'-.',1:time,X_attack(3,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\theta$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%�����������С
subplot(2,2,4)        
plot(1:time,Xkf(4,:),'-',1:time,hat_X(4,:),'-.',1:time,X_attack(4,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\omega$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%�����������С