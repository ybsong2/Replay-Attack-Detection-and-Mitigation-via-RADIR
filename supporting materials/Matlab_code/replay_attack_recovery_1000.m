%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序用于产生非连续的重放攻击时长
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear,clc
%% 本部分确定的攻击时长范围为[0-11]


max_attacktime=12;
f=[0.2000 0.1637 0.1341 0.1098 0.0899 0.0736 0.0602 0.0493 0.0404 0.0331 0.0271 0.0188];
extra_length=[0:11];

basic_length=10;%设置基本攻击时长
split=10;        %设置重放攻击的片段数
attack_length_set=[];%存放攻击各个片段的攻击时长
attack_start_set=[];%存放各个片段的攻击起始时刻
for i=1:split
    attack_length=randsrc(1,1,[extra_length;f]);%当前时刻的攻击时长
    attack_length_set=[attack_length_set attack_length+basic_length];
    attack_start=randi([50*i 50*i+25],1,1);         %这里可以改变起始时刻t_0的选择
    attack_start_set=[attack_start_set attack_start];
end
time=300;
replay_attack=zeros(1,time);%用于存放重放攻击
for j=1:split
    for i=1:time
        if i>=attack_start_set(j) & i<attack_start_set(j)+attack_length_set(j)
            replay_attack(i)=1;
        end
    end
end
%plot(1:time,replay_attack,'-');


%% 下面是系统运行过程，系统运行时间进行了单独设计
N=time; %仿真时间，时间序列总数

%% 噪声
duijiao2=[0.131 0.123];
Q=diag([0.228 0.0102 0.683 0.0109]);%过程噪声方差为0，即下落过程忽略空气阻力
R=diag(duijiao2); %观测噪声方差
W=sqrt(Q)*randn(4,N);%既然Q为0，即W=0
V=sqrt(R)*randn(2,N);%测量噪声V(k)

%% 系数矩阵
A=[1.0000,0.0100,0.0000,0.0000;0.0000,0.9950,0.0000,0.0000;0.0000,0.0000,1.00000,0.0100;0.0000,0.0000,0.0000,0.9950];%状态转移矩阵
B=[2.49583853646267e-05,0;0.00498752080731769,0;0,2.49583853646267e-05;0,0.00498752080731769];                       %输入矩阵
U=[0;0];
H=[1 0 0 0;0 0 1 0];%观测矩阵

%% 初始化
X=zeros(4,N);%物体真实状态
X(:,1)=[3;1;3;1];%初始位移和速度
P0=[0.0931 0.0040 0 0;0.0040 0.9809 0 0;0 0 0.1064 0.0017;0 0 0.0017 1.0759];%初始误差
%Z=zeros(1,N);
Z=zeros(2,N);
%Z(1)=H*X(:,1);%初始观测值
Z(:,1)=H*X(:,1);
Xkf=zeros(4,N);%卡尔曼估计状态初始化
Xkf(:,1)=X(:,1);
X_pre_set=zeros(4,N);%卡尔曼先验状态的初始化
X_pre_set(:,1)=X(:,1);
err_P=zeros(N,4);

%% 系统运行过程
I=eye(4); %4维系统
for k=2:N
    X(:,k)=A*X(:,k-1)+B*U+W(k);         %物体下落，受状态方程的驱动
    Z(:,k)=H*X(:,k)+V(k);               %位移传感器对目标进行观测
    %卡尔曼滤波
    X_pre=A*Xkf(:,k-1)+B*U;             %状态预测 
    X_pre_set(:,k)=X_pre;
    P_pre=A*P0*A'+Q;                    %协方差预测
    Kg=P_pre*H'*inv(H*P_pre*H'+R);      %计算卡尔曼增益
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);   %状态更新
    P0=(I-Kg*H)*P_pre;                  %方差更新
end



time=300;%运行次数
p=10;%设置周期长度为10
period_policy_number=8;%设置8个不同的调度策略
encryption_time_pro=[0.01 0.04 0.15 0.3 0.3 0.15 0.04 0.01];%单个周期内2-9次加密的概率
policy=cell(1,period_policy_number);
%%%最优加密调度策略
policy{1}=[0 0 0 0 1 0 0 0 0 1];
policy{2}=[0 0 1 0 0 0 1 0 0 1];
policy{3}=[0 1 0 0 1 0 1 0 0 1];
policy{4}=[0 1 0 1 0 1 0 1 0 1];
policy{5}=[0 1 0 1 1 0 1 0 1 1];
policy{6}=[0 1 1 0 1 1 1 0 1 1];
policy{7}=[0 1 1 1 1 0 1 1 1 1];
policy{8}=[0 1 1 1 1 1 1 1 1 1];
%%%集中加密调度策略
policy_center{1}=[0 0 0 0 0 0 0 0 1 1];
policy_center{2}=[0 0 0 0 0 0 0 1 1 1];
policy_center{3}=[0 0 0 0 0 0 1 1 1 1];
policy_center{4}=[0 0 0 0 0 1 1 1 1 1];
policy_center{5}=[0 0 0 0 1 1 1 1 1 1];
policy_center{6}=[0 0 0 1 1 1 1 1 1 1];
policy_center{7}=[0 0 1 1 1 1 1 1 1 1];
policy_center{8}=[0 1 1 1 1 1 1 1 1 1];
for k=1:time
    %%%%确定最优加密调度策略
    if mod(k-1,p)==0
       encryption_time=randsrc(1,1,[[2:9];encryption_time_pro]);%当前时刻的加密次数
       gamma(k:k+p-1)=policy{encryption_time-1};
       gamma_center(k:k+p-1)=policy_center{encryption_time-1};
       %%%完成随机加密调度策略
       random_set=randperm(p);
       for i=1:p
           if random_set(i)<=encryption_time
               random_01_set(i)=1;
           else
               random_01_set(i)=0;
           end
       end
       gamma_random(k:k+p-1)=random_01_set;%随机策略集合
    end
    %%%%为加密点设置标签
    if gamma(k)==1
        label(k)=k;
        gamma_point(k)=1;
    else 
        label(k)=0;
        gamma_point(k)=NaN;
    end
    %%%%当攻击来临时，加密点的标签被重新设置
    if replay_attack(k)==1
        if gamma(k)==1
            label(k)=10000;%设置较大的标签量10000，可以更改
        end
    end    
end
%% 重放攻击恢复算法的状态更新过程
attack_exist=0;
tau=0;
attack_inter_set=zeros(1,time);
P_error_matrix=cell(1,time);
for k=1:time
    if gamma(k)==1
        if label(k)==k
            hat_X(:,k)=Xkf(:,k);
            P_error_matrix{k}=P0;
            tau=k;%用于存储上一个加密点所对应的时刻值
            attack_exist=0;%表示当前时刻没有攻击
            attack_inter_set(k)=attack_exist;
        else
            hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%表示当前
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
%% 模拟重放攻击
replay_attack_start=10;
X_attack=Xkf;%用于存放重放攻击后的状态值
for j=1:split
    for i=1:time
        if i>=attack_start_set(j) & i<attack_start_set(j)+attack_length_set(j)
            X_attack(:,i)=Xkf(:,replay_attack_start+i-attack_start_set(j));  %只把存在攻击部分的数据进行相应的替换
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
set(gca,'FontSize',18);%坐标轴字体大小
subplot(2,2,2)        
plot(1:time,Xkf(2,:),'-',1:time,hat_X(2,:),'-.',1:time,X_attack(2,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$v$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小
subplot(2,2,3)        
plot(1:time,Xkf(3,:),'-',1:time,hat_X(3,:),'-.',1:time,X_attack(3,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\theta$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小
subplot(2,2,4)        
plot(1:time,Xkf(4,:),'-',1:time,hat_X(4,:),'-.',1:time,X_attack(4,:),'--','linewidth',2);
legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\omega$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小