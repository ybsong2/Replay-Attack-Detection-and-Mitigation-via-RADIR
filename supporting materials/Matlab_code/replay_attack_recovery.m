%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������طŹ�����������ָ��㷨����ִ��˳����replayattack_timespan����>������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%% ���ڱ�ǩ���ԭ���ҵ���ȷ���Ĺ������ڵ�����
attack_position=find(attack_inter_set>=1);%���ܵ��⵽������λ��
attack_start=[attack_position(1)];
    for i=1:size(attack_position,2)-1
        if attack_position(i+1)-attack_position(i)>p
            attack_start=[attack_start attack_position(i) attack_position(i+1)];
        end
    end
attack_start=[attack_start attack_position(size(attack_position,2))];%������ʼ-��ֹλ�ü���
attack_interval=zeros(1,time);%���ڴ��ȷ���������ڵ�����
for j=1:2:size(attack_start,2)
    for i=1:time    
        if i>=attack_start(j) && i<=attack_start(j+1)
            attack_interval(i)=1;
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

%% ���ƹ���ǰ���״̬ͼ��
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

%% ���ƹ������䣬������䣬���ܵ�ķֲ����
figure          %���ƹ������䣬������䣬���ܵ�ķֲ����
subplot(2,1,1)
stairs(replay_attack,'-','linewidth',2);
hold on
stairs(attack_interval,'-.','linewidth',2);
set(gca,'YTick',[0:1:1]);%����Ҫ��ʾ����̶�
legend({'Replay attack','\emph{Label Detection}',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northwest');
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\mathcal{A}(t_0, T_r)/\left[t^\star_{start},t^\star_{last}\right]$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%�����������С
subplot(2,1,2)
stem(1:time,gamma_point,'linewidth',1);
set(gca,'YTick',[0:1:1]);%����Ҫ��ʾ����̶�
xlabel('Time' ,'Interpreter','latex','FontSize',18);
ylabel({'$\gamma_k$'}, 'Interpreter','latex','FontSize',18);
set(gca,'FontSize',18);%�����������С

