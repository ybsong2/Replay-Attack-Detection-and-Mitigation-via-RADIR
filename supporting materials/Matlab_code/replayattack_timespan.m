%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序用于产生非连续的重放攻击时长
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear,clc
%% 本部分确定的攻击时长范围为[0-11]
attack_X=10;
xi=1/5;
x=0:1:attack_X;
for i=1:attack_X+1
    f(i)=xi*exp(-xi*x(i));%概率密度函数（参考Optimal periodic watermarking schedule）
end
%plot(x,f)
max_attacktime=size([f 1-sum(f)],2);%确定最大的攻击时长为max_attacktime-1

basic_length=10;%设置基本攻击时长
split=5;        %设置重放攻击的片段数
attack_length_set=[];%存放攻击各个片段的攻击时长
attack_start_set=[];%存放各个片段的攻击起始时刻
for i=1:split
    attack_length=randsrc(1,1,[[0:max_attacktime-1];[f 1-sum(f)]]);%当前时刻的攻击时长
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
X(:,1)=[1;1;2;3];%初始位移和速度
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

%% 本部分确定的攻击时长范围为[0-98]
% attack_X=97;
% xi=1/25;
% x=0:1:attack_X;
% for i=1:attack_X+1
%     f(i)=xi*exp(-xi*x(i));%概率密度函数（参考Optimal periodic watermarking schedule）
% end
% %plot(x,f)
% max_attacktime=size([f 1-sum(f)],2);%确定最大的攻击时长为max_attacktime-1
% attack_length=randsrc(1,1,[[0:max_attacktime-1];[f 1-sum(f)]]);%当前时刻的攻击时长