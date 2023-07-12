%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%本程序用于验证加密比率与攻击检测性之间的关系
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear,clc

repeat_times=1000;
p_detection_smrepeat=zeros(10,repeat_times);
for m=1:repeat_times
%% 本部分确定的攻击时长范围为[0-11]
attack_X=10;
xi=1/5;
x=0:1:attack_X;
for i=1:attack_X+1
    f(i)=xi*exp(-xi*x(i));%概率密度函数（参考Optimal periodic watermarking schedule）
end
%plot(x,f)
max_attacktime=size([f 1-sum(f)],2);%确定最大的攻击时长为max_attacktime-1

basic_length=5;%设置基本攻击时长
split=1;        %设置重放攻击的片段数
attack_length_set=[];%存放攻击各个片段的攻击时长
attack_start_set=[];%存放各个片段的攻击起始时刻
for i=1:split
    attack_length=randsrc(1,1,[[0:max_attacktime-1];[f 1-sum(f)]]);%当前时刻的攻击时长
    attack_length_set=[attack_length_set attack_length+basic_length];
    attack_start=randi([50*i 50*i+25],1,1);         %这里可以改变起始时刻t_0的选择
    attack_start_set=[attack_start_set attack_start];
end

p=10;%设置周期长度为10
%% 当攻击仅位于单个调度周期内，p_detection存放检测概率
feiyi=p-attack_length_set;
if feiyi>0
    for i=1:feiyi
        p_detection(i)=1-(factorial(p-i)/(factorial(p-i-attack_length_set))/(factorial(p)/factorial(p-attack_length_set)));
    end
    for i=feiyi+1:p
        p_detection(i)=1;
    end
else
    for i=1:p
        p_detection(i)=1;
    end
end

%% 当攻击跨越两个调度周期时,p_detection_span存放检测概率
feiyi_span=p-ceil(attack_length_set/2);
if feiyi_span>0
    for i=1:feiyi_span
        p_detection_span(i)=1-(factorial(p-i)/(factorial(p-i-ceil(attack_length_set/2)))/(factorial(p)/factorial(p-ceil(attack_length_set/2))))...
             *(factorial(p-i)/(factorial(p-i-attack_length_set+ceil(attack_length_set/2)))/(factorial(p)/factorial(p-attack_length_set+ceil(attack_length_set/2))));
    end
    for i=feiyi_span+1:p
        p_detection_span(i)=1;
    end
else
    for i=1:p
        p_detection_span(i)=1;
    end
end

p_detection_smrepeat(:,m)=p_detection;
p_detection_span_smrepeat(:,m)=p_detection_span;
end
for i=1:10
    p_detection_smrepeat_aver(i)=sum(p_detection_smrepeat(i,:))/repeat_times;
    p_detection_span_smrepeat_aver(i)=sum(p_detection_span_smrepeat(i,:))/repeat_times;
end
subplot(2,1,1)        
%plot(0:0.1:1,[0,p_detection],'-^','linewidth',2);
plot(0:0.1:1,[0,p_detection_smrepeat_aver],'-^','linewidth',2);
%legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Encryption ratio' ,'Interpreter','latex','FontSize',18);
ylabel({'$P_d$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小
title('Case $1)$ in $Theorem \ 1$','Interpreter','latex','FontSize',18);
subplot(2,1,2)        
%plot(0:0.1:1,[0,p_detection_span],'-o','linewidth',2);
plot(0:0.1:1,[0,p_detection_span_smrepeat_aver],'-o','linewidth',2);
%legend({'$\hat{x}_k^s$','$\hat{x}_k$','$\hat{x}_k^a$',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
xlabel('Encryption ratio' ,'Interpreter','latex','FontSize',18);
ylabel({'$P_d$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%坐标轴字体大小
title('Case $2)$ in $Theorem \ 1$','Interpreter','latex','FontSize',18);


