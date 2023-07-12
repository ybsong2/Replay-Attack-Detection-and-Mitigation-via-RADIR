%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序用于模拟重放攻击的实现过程
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc,clear
N=1000; %仿真时间，时间序列总数


Q=[1,0;0,1];%过程噪声方差为0，即下落过程忽略空气阻力
R=[1 0;0 1];
A=[-0.0022 -0.0030; 1.0625 0.9808];%状态转移矩阵
B=[0;0];%控制量
U=-1;
H=[0 0.170667;0 0];%观测矩阵
W=sqrt(Q)*randn(2,N);%既然Q为0，即W=0
V=sqrt(R)*randn(2,N);

%a=[0.9394 0 0.0575 0;0 0.9508 0 0.0496;0 0 0.9406 0;0 0 0 0.9491]

%% %% 闭环事件触发调度
%初始化
X=zeros(2,N);%物体真实状态
X(:,1)=[0;0];%初始位移和速度
%P0=[1.3944 -0.0176;-0.0176 0.3528];
P0=[1.0001 -0.0183;-0.0183 7.0516];
%Z=zeros(1,N);
y=zeros(2,N);
%Z(1)=H*X(:,1);%初始观测值
y(:,1)=H*X(:,1);
Xkf=zeros(2,N);%卡尔曼估计状态初始化
Xkf(:,1)=X(:,1);
err_P=zeros(N,2);
err_P(1,1)=P0(1,1);
err_P(1,2)=P0(2,2);
I=eye(2); %二维系统
for k=2:N
    X(:,k)=A*X(:,k-1)+B*U+W(k);         %物体下落，受状态方程的驱动
    Z(:,k)=H*X(:,k)+V(k);               %位移传感器对目标进行观测
    %卡尔曼滤波
    X_pre=A*Xkf(:,k-1)+B*U;             %状态预测 
    P_pre=A*P0*A'+Q;                    %协方差预测
    Kg=P_pre*H'*inv(H*P_pre*H'+R);      %计算卡尔曼增益
    Xkf(:,k)=X_pre+Kg*(Z(k)-H*X_pre);   %状态更新
    P0=(I-Kg*H)*P_pre;                  %方差更新
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