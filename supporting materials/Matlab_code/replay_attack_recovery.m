%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序是重放攻击检测隔离与恢复算法，其执行顺序是replayattack_timespan——>本程序
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%% 基于标签检测原理，找到可确定的攻击存在的区间
attack_position=find(attack_inter_set>=1);%加密点检测到攻击的位置
attack_start=[attack_position(1)];
    for i=1:size(attack_position,2)-1
        if attack_position(i+1)-attack_position(i)>p
            attack_start=[attack_start attack_position(i) attack_position(i+1)];
        end
    end
attack_start=[attack_start attack_position(size(attack_position,2))];%攻击起始-终止位置集合
attack_interval=zeros(1,time);%用于存放确定攻击存在的区间
for j=1:2:size(attack_start,2)
    for i=1:time    
        if i>=attack_start(j) && i<=attack_start(j+1)
            attack_interval(i)=1;
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

%% 绘制攻击前后的状态图形
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

%% 绘制攻击区间，检测区间，加密点的分布情况
figure          %绘制攻击区间，检测区间，加密点的分布情况
subplot(2,1,1)
stairs(replay_attack,'-','linewidth',2);
hold on
stairs(attack_interval,'-.','linewidth',2);
set(gca,'YTick',[0:1:1]);%设置要显示坐标刻度
legend({'Replay attack','\emph{Label Detection}',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northwest');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\mathcal{A}(t_0, T_r)/\left[t^\star_{start},t^\star_{last}\right]$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小
subplot(2,1,2)
stem(1:time,gamma_point,'linewidth',1);
set(gca,'YTick',[0:1:1]);%设置要显示坐标刻度
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\gamma_k$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小

