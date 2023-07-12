%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序对检测的及时性性能进行比较（最优调度、集中调度和随机调度），进行了1000次仿真求平均值
%split=10表示发起10子攻击，time=550表示每隔50s发起一次重放攻击
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序是基于DI_comparation工作区的数据而得到的，这是最开始仿真图
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% optimal_DI=[1 1 1 2 3];
% center_DI=[2 2 6 11 21];
% random_DI=[2 2 3 4 10];
% plot(1:split,optimal_DI,'-^',1:split,center_DI,'-.>',1:split,random_DI,'--o','linewidth',2);
% legend({'Optimal encryption scheduling','Centralized encryption scheduling','Random encryption scheduling',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
% xlabel('Times of subattack' ,'Interpreter','latex','FontSize',18);
% ylabel({'$DI\left(T_r,\theta\right)$'}, 'Interpreter','latex','FontSize',18);
% set(gca,'FontSize',18);%坐标轴字体大小
% set(gca,'XTick',[1:1:5]);%设置要显示坐标刻度



%% 1000次重复执行过程
clear,clc

repeat_times=1;
split=10;
optimal_DI_cunchu=zeros(split,repeat_times);
center_DI_cunchu=zeros(split,repeat_times);
random_DI_cunchu=zeros(split,repeat_times);
for m=1:repeat_times
%% 本部分确定的攻击时长范围为[0-11]
% attack_X=10;
% xi=1/5;
% x=0:1:attack_X;
% for i=1:attack_X+1
%     f(i)=xi*exp(-xi*x(i));%概率密度函数（参考Optimal periodic watermarking schedule）
% end

%max_attacktime=size([f 1-sum(f)],2);%确定最大的攻击时长为max_attacktime-1
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
time=550;
replay_attack=zeros(1,time);%用于存放重放攻击
for j=1:split
    for i=1:time
        if i>=attack_start_set(j) & i<attack_start_set(j)+attack_length_set(j)
            replay_attack(i)=1;
        end
    end
end


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
    %%%%为集中调度策略设置标签
    if gamma_center(k)==1;
        label_center(k)=k;
        gamma_point_center(k)=1;
    else
        label_center(k)=0;
        gamma_point_center(k)=NaN;
    end
    %%%%为随机调度策略设置标签
    if gamma_random(k)==1;
        label_random(k)=k;
        gamma_point_random(k)=1;
    else
        label_random(k)=0;
        gamma_point_random(k)=NaN;
    end
    %%%%当攻击来临时，加密点的标签被重新设置
    if replay_attack(k)==1
        if gamma(k)==1
            label(k)=10000;%设置较大的标签量10000，可以更改
            
        end
        if gamma_center(k)==1
            label_center(k)=10000;
        end
        if gamma_random(k)==1
            label_random(k)=10000;
        end
    end    
end
%% 重放攻击恢复算法的状态更新过程
attack_exist=0;
tau=0;
attack_inter_set=zeros(1,time);
%P_error_matrix=cell(1,time);
for k=1:time
    if gamma(k)==1
        if label(k)==k
            %hat_X(:,k)=Xkf(:,k);
            %P_error_matrix{k}=P0;
            tau=k;%用于存储上一个加密点所对应的时刻值
            attack_exist=0;%表示当前时刻没有攻击
            attack_inter_set(k)=attack_exist;
        else
            %hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            %P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%表示当前
            attack_inter_set(k)=attack_exist;
        end
    end        
end

%集中调度过程
attack_exist=0;
tau=0;
attack_inter_set_center=zeros(1,time);
%P_error_matrix=cell(1,time);
for k=1:time
    if gamma_center(k)==1
        if label_center(k)==k
            %hat_X(:,k)=Xkf(:,k);
            %P_error_matrix{k}=P0;
            tau=k;%用于存储上一个加密点所对应的时刻值
            attack_exist=0;%表示当前时刻没有攻击
            attack_inter_set_center(k)=attack_exist;
        else
            %hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            %P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%表示当前
            attack_inter_set_center(k)=attack_exist;
        end
    end        
end
%随机调度过程
attack_exist=0;
tau=0;
attack_inter_set_random=zeros(1,time);
%P_error_matrix=cell(1,time);
for k=1:time
    if gamma_random(k)==1
        if label_random(k)==k
            %hat_X(:,k)=Xkf(:,k);
            %P_error_matrix{k}=P0;
            tau=k;%用于存储上一个加密点所对应的时刻值
            attack_exist=0;%表示当前时刻没有攻击
            attack_inter_set_random(k)=attack_exist;
        else
            %hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            %P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%表示当前
            attack_inter_set_random(k)=attack_exist;
        end
    end        
end

%% 基于标签检测原理，找到可确定的攻击存在的区间
attack_position=find(attack_inter_set>=1);%加密点检测到攻击的位置
attack_position_center=find(attack_inter_set_center>=1);
attack_position_random=find(attack_inter_set_random>=1);

for i=1:split%用于确定单个子攻击的时间间隔
    position(i)=0;
    for j=1:size(attack_position,2)
        if attack_position(j)>=attack_start_set(i) & attack_position(j)<attack_start_set(i)+attack_length_set(i)
            position(i)=position(i)+1;
        end
    end
    position_center(i)=0;
    for j=1:size(attack_position_center,2)
        if attack_position_center(j)>=attack_start_set(i) & attack_position_center(j)<attack_start_set(i)+attack_length_set(i)
            position_center(i)=position_center(i)+1;
        end
    end
    position_random(i)=0;
    for j=1:size(attack_position_random,2)
        if attack_position_random(j)>=attack_start_set(i) & attack_position_random(j)<attack_start_set(i)+attack_length_set(i)
            position_random(i)=position_random(i)+1;
        end
    end
end
sum=0;
sum_center=0;
sum_random=0;
for i=1:split  
    sum_position(i)=sum+position(i);
    sum=sum_position(i);
    sum_position_center(i)=sum_center+position_center(i);
    sum_center=sum_position_center(i);
    sum_position_random(i)=sum_random+position_random(i);
    sum_random=sum_position_random(i);
end
jiange(1)=attack_position(sum_position(1))-attack_position(1)+1;
jiange_center(1)=attack_position_center(sum_position_center(1))-attack_position_center(1)+1;
jiange_random(1)=attack_position_random(sum_position_random(1))-attack_position_random(1)+1;
for i=2:split
    jiange(i)=attack_position(sum_position(i))-attack_position(sum_position(i-1)+1)+1;
    jiange_center(i)=attack_position_center(sum_position_center(i))-attack_position_center(sum_position_center(i-1)+1)+1;
    jiange_random(i)=attack_position_random(sum_position_random(i))-attack_position_random(sum_position_random(i-1)+1)+1;
end
error=attack_length_set-jiange;%求得不可检测时刻值
error_center=attack_length_set-jiange_center;
error_random=attack_length_set-jiange_random;
sum=0;
sum_center=0;
sum_random=0;
for i=1:split  
    optimal_DI(i)=sum+error(i);
    sum=optimal_DI(i);
    center_DI(i)=sum_center+error_center(i);
    sum_center=center_DI(i);
    random_DI(i)=sum_random+error_random(i);
    sum_random=random_DI(i);
end
optimal_DI_cunchu(:,m)=optimal_DI;
center_DI_cunchu(:,m)=center_DI;
random_DI_cunchu(:,m)=random_DI;
end
for i=1:split
    sum=0;
    sum_center=0;
    sum_random=0;
    for j=1:repeat_times
        optimal_DI_cunchu_sum=sum+optimal_DI_cunchu(i,j);
        sum=optimal_DI_cunchu_sum;
        center_DI_cunchu_sum=sum_center+center_DI_cunchu(i,j);
        sum_center=center_DI_cunchu_sum;
        random_DI_cunchu_sum=sum_random+random_DI_cunchu(i,j);
        sum_random=random_DI_cunchu_sum;      
    end
    optimal_DI_cunchu_aver(i)=(optimal_DI_cunchu_sum)/repeat_times;
    center_DI_cunchu_aver(i)=(center_DI_cunchu_sum)/repeat_times;
    random_DI_cunchu_aver(i)=(random_DI_cunchu_sum)/repeat_times;
end

figure
plot(1:split,optimal_DI_cunchu_aver,'-^',1:split,center_DI_cunchu_aver,'-.>',1:split,random_DI_cunchu_aver,'--o','linewidth',2);
legend({'Optimal encryption scheduling','Centralized encryption scheduling','Random encryption scheduling',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Times of subattack' ,'Interpreter','latex','FontSize',18);
ylabel({'$DI\left(T_r,\theta\right)$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小
set(gca,'XTick',[1:1:10]);%设置要显示坐标刻度



% attack_start=[attack_position(1)];
% attack_start_center=[attack_position_center(1)];
% attack_start_random=[attack_position_random(1)];
%     for i=1:size(attack_position,2)-1
%         if attack_position(i+1)-attack_position(i)>p
%             attack_start=[attack_start attack_position(i) attack_position(i+1)];
%         end
%     end
%     for i=1:size(attack_position_center,2)-1
%         if attack_position_center(i+1)-attack_position_center(i)>p
%             attack_start_center=[attack_start_center attack_position_center(i) attack_position_center(i+1)];
%         end
%     end
%     for i=1:size(attack_position_random,2)-1
%         if attack_position_random(i+1)-attack_position_random(i)>p
%             attack_start_random=[attack_start_random attack_position_random(i) attack_position_random(i+1)];
%         end
%     end
% attack_start=[attack_start attack_position(size(attack_position,2))];%攻击起始-终止位置集合
% attack_start_center=[attack_start_center attack_position_center(size(attack_position,2))];
% attack_start_random=[attack_start_random attack_position_random(size(attack_position,2))];


% attack_interval=zeros(1,time);%用于存放确定攻击存在的区间
% for j=1:2:size(attack_start,2)
%     for i=1:time    
%         if i>=attack_start(j) && i<=attack_start(j+1)
%             attack_interval(i)=1;
%         end
%     end
% end

