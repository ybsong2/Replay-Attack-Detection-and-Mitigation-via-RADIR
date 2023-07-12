%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序用于模拟重放攻击的实现过程
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear;

N=1000; %仿真时间，时间序列总数
%N=10000;
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


t_record_start=11;
t_start=51;
t_end=70;
for k=1:N
    if k>=t_start&k<=t_end
        X_attack(:,k)=Xkf(:,t_record_start+k-t_start);
        X_recovery(:,k)=A^(k-t_start-1)*X_pre_set(:,t_start-1);
    else
        X_attack(:,k)=Xkf(:,k);
        X_recovery(:,k)=Xkf(:,k);
    end
end
plot(1:N,X_attack(1,:),'-',1:N,Xkf(1,:),'--',1:N,X_recovery(1,:),'-.');
figure
plot(1:N,X_attack(2,:),'-',1:N,Xkf(2,:),'--',1:N,X_recovery(2,:),'-.');

