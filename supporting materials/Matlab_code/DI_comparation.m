%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������Լ��ļ�ʱ�����ܽ��бȽϣ����ŵ��ȡ����е��Ⱥ�������ȣ���������1000�η�����ƽ��ֵ
%split=10��ʾ����10�ӹ�����time=550��ʾÿ��50s����һ���طŹ���
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�������ǻ���DI_comparation�����������ݶ��õ��ģ������ʼ����ͼ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% optimal_DI=[1 1 1 2 3];
% center_DI=[2 2 6 11 21];
% random_DI=[2 2 3 4 10];
% plot(1:split,optimal_DI,'-^',1:split,center_DI,'-.>',1:split,random_DI,'--o','linewidth',2);
% legend({'Optimal encryption scheduling','Centralized encryption scheduling','Random encryption scheduling',},  'Interpreter','latex', 'FontSize', 18, 'location', 'northeast');
% xlabel('Times of subattack' ,'Interpreter','latex','FontSize',18);
% ylabel({'$DI\left(T_r,\theta\right)$'}, 'Interpreter','latex','FontSize',18);
% set(gca,'FontSize',18);%�����������С
% set(gca,'XTick',[1:1:5]);%����Ҫ��ʾ����̶�



%% 1000���ظ�ִ�й���
clear,clc

repeat_times=1;
split=10;
optimal_DI_cunchu=zeros(split,repeat_times);
center_DI_cunchu=zeros(split,repeat_times);
random_DI_cunchu=zeros(split,repeat_times);
for m=1:repeat_times
%% ������ȷ���Ĺ���ʱ����ΧΪ[0-11]
% attack_X=10;
% xi=1/5;
% x=0:1:attack_X;
% for i=1:attack_X+1
%     f(i)=xi*exp(-xi*x(i));%�����ܶȺ������ο�Optimal periodic watermarking schedule��
% end

%max_attacktime=size([f 1-sum(f)],2);%ȷ�����Ĺ���ʱ��Ϊmax_attacktime-1
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
time=550;
replay_attack=zeros(1,time);%���ڴ���طŹ���
for j=1:split
    for i=1:time
        if i>=attack_start_set(j) & i<attack_start_set(j)+attack_length_set(j)
            replay_attack(i)=1;
        end
    end
end


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
    %%%%Ϊ���е��Ȳ������ñ�ǩ
    if gamma_center(k)==1;
        label_center(k)=k;
        gamma_point_center(k)=1;
    else
        label_center(k)=0;
        gamma_point_center(k)=NaN;
    end
    %%%%Ϊ������Ȳ������ñ�ǩ
    if gamma_random(k)==1;
        label_random(k)=k;
        gamma_point_random(k)=1;
    else
        label_random(k)=0;
        gamma_point_random(k)=NaN;
    end
    %%%%����������ʱ�����ܵ�ı�ǩ����������
    if replay_attack(k)==1
        if gamma(k)==1
            label(k)=10000;%���ýϴ�ı�ǩ��10000�����Ը���
            
        end
        if gamma_center(k)==1
            label_center(k)=10000;
        end
        if gamma_random(k)==1
            label_random(k)=10000;
        end
    end    
end
%% �طŹ����ָ��㷨��״̬���¹���
attack_exist=0;
tau=0;
attack_inter_set=zeros(1,time);
%P_error_matrix=cell(1,time);
for k=1:time
    if gamma(k)==1
        if label(k)==k
            %hat_X(:,k)=Xkf(:,k);
            %P_error_matrix{k}=P0;
            tau=k;%���ڴ洢��һ�����ܵ�����Ӧ��ʱ��ֵ
            attack_exist=0;%��ʾ��ǰʱ��û�й���
            attack_inter_set(k)=attack_exist;
        else
            %hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            %P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%��ʾ��ǰ
            attack_inter_set(k)=attack_exist;
        end
    end        
end

%���е��ȹ���
attack_exist=0;
tau=0;
attack_inter_set_center=zeros(1,time);
%P_error_matrix=cell(1,time);
for k=1:time
    if gamma_center(k)==1
        if label_center(k)==k
            %hat_X(:,k)=Xkf(:,k);
            %P_error_matrix{k}=P0;
            tau=k;%���ڴ洢��һ�����ܵ�����Ӧ��ʱ��ֵ
            attack_exist=0;%��ʾ��ǰʱ��û�й���
            attack_inter_set_center(k)=attack_exist;
        else
            %hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            %P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%��ʾ��ǰ
            attack_inter_set_center(k)=attack_exist;
        end
    end        
end
%������ȹ���
attack_exist=0;
tau=0;
attack_inter_set_random=zeros(1,time);
%P_error_matrix=cell(1,time);
for k=1:time
    if gamma_random(k)==1
        if label_random(k)==k
            %hat_X(:,k)=Xkf(:,k);
            %P_error_matrix{k}=P0;
            tau=k;%���ڴ洢��һ�����ܵ�����Ӧ��ʱ��ֵ
            attack_exist=0;%��ʾ��ǰʱ��û�й���
            attack_inter_set_random(k)=attack_exist;
        else
            %hat_X(:,k)=A^(k-tau)*Xkf(:,tau);
            %P_error_matrix{k}=A^(k-tau)*P0*(A')^(k-tau);
            attack_exist=1;%��ʾ��ǰ
            attack_inter_set_random(k)=attack_exist;
        end
    end        
end

%% ���ڱ�ǩ���ԭ���ҵ���ȷ���Ĺ������ڵ�����
attack_position=find(attack_inter_set>=1);%���ܵ��⵽������λ��
attack_position_center=find(attack_inter_set_center>=1);
attack_position_random=find(attack_inter_set_random>=1);

for i=1:split%����ȷ�������ӹ�����ʱ����
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
error=attack_length_set-jiange;%��ò��ɼ��ʱ��ֵ
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
set(gca,'FontSize',18);%�����������С
set(gca,'XTick',[1:1:10]);%����Ҫ��ʾ����̶�



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
% attack_start=[attack_start attack_position(size(attack_position,2))];%������ʼ-��ֹλ�ü���
% attack_start_center=[attack_start_center attack_position_center(size(attack_position,2))];
% attack_start_random=[attack_start_random attack_position_random(size(attack_position,2))];


% attack_interval=zeros(1,time);%���ڴ��ȷ���������ڵ�����
% for j=1:2:size(attack_start,2)
%     for i=1:time    
%         if i>=attack_start(j) && i<=attack_start(j+1)
%             attack_interval(i)=1;
%         end
%     end
% end

