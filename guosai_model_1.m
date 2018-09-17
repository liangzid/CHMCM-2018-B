function [nums,product_number,begin_time,end_time]=guosai_model_1(group)
%% ����˵��
%t_shift����������ʾ�ƶ�i����λ����ʱ��
%t_product:��ʾ��������ʱ��
%t_ud


%% ��ʼ��
%��������ʱ��Ĳ����趨
t_shift=[0 0 0];
t_product=0;
t_up=[0 0];
t_clean=0;
if group==1
    t_shift=[20 33 46];
    t_product=560;
    t_up=[28 31];
    t_clean=25;
    else if group==2
        t_shift=[23 41 59];
        t_product=580;
        t_up=[30 35];
        t_clean=30;
        else if group==3
            t_shift=[18 32 46];
            t_product=545;
            t_up=[27 32];
            t_clean=25;
            end
        end
end
%��ʼ��״̬����ģʽ����Ϊ��ʼ��״̬��
M=ones(1,8);
%��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
need_time=zeros(1,8);
%��ʼ������ʱ��
all_time=8*3600;
cost_time=0;
%��ʼ��RGV��λ��
RGV_location=1;
%��ʼ���ܹ����������������
nums=0;
%��ʼ������2״̬��CNC�ļӹ��ۼ�ʱ��
TM=zeros(1,8);

%����ӹ����
product_number=[];
%���������ϵĿ�ʼʱ��
begin_time=[];
end_time=[];

qwqw=1;
%% ��ʼ����ѭ�����൱��������������
while cost_time<=all_time
    %�����ڱ����ֻ������ĵ�ʱ��
    per_cost=0;
    wait_time=0;
    shift_time=0;
    change_where=RGV_location;
    numss=0;
    clean_time=0;
    shift_time=0;
    waste_time=0;
    %RGV����
    if guosai_is_need_action(M)
        need_time=guosai_time_loss(RGV_location,M,t_shift);%��������ʱ��-------------------------------------0
        %����Ӧ����ô�ߣ��������˵ĵ���״ֵ̬�����ƶ�����λ�á��ƶ������ʱ�䣻���д���������֮��RGV���Ѿ��˶���
        [change_where,M1,shift_time]=guosai_decision_tanxin(M,need_time,t_up);%--------------------------1
        
        % % ��ȡ�ӹ���CNC��ţ��Լ������ϵĿ�ʼʱ��
        if M(change_where)==1%�����ʼ���׶Σ�������
            product_number=[product_number,change_where];
            begin_time=[begin_time,(need_time(change_where)+cost_time)];
        else  %������������״̬3�����Թ�ȥ���Ͻ������ϣ���δ�����Ͻ�������
            product_number=[product_number,change_where];
            end_time=[end_time,(need_time(change_where)+cost_time)];
            begin_time=[begin_time,(need_time(change_where)+cost_time)];
        end
        
        %��ϴ�Ĺ���
        if M(change_where)==3
            clean_time=t_clean;
        end
        shift_time=shift_time+clean_time;
        %���˶�ʱ���ϵĴ���2״̬��CNC�ļӹ�ʱ��
        [TM,M1]=guosai_time_shift_go(M,M1,shift_time,TM,t_product);
        %ͳ���ڽ����˶�֮���Ƿ�����������˼ӹ�����
        numss=guosai_num_of_finish(M,M1);%----------------------------------------------------------0
      
    %����û������RGV�ȴ�
    else
        [TM,M1,wait_time]=guosai_wait(M,TM,t_product);
    end
    %���㾭�������˶�֮����һ�ο��Կ�ʼ�����˶�֮ǰ����ʱ��    
    per_cost=wait_time+shift_time;
    
    
    %���е���֮��ĸ���
    RGV_location=change_where;
    M=M1;
    cost_time=cost_time+per_cost;
    nums=nums+numss;
    qwqw=qwqw+1;

end
%��������
guosai_conduct_data(product_number,begin_time,end_time,'D:\desktop\test.xls');



%% ̰���㷨����
    function [change_where,M,shift_time]=guosai_decision_tanxin(M,need_time,t_up)
        decision_index=1;
        for gdt=1:8
            if (need_time(gdt)+guosai_time_up(gdt,t_up))<=(need_time(decision_index)+guosai_time_up(decision_index,t_up))
                decision_index=gdt;
            end
        end
        change_where=decision_index;
        M=guosai_mode_change(M,change_where);
        shift_time=need_time(decision_index)+guosai_time_up(decision_index,t_up);
    end

%% ��������������֮��Ļ���
%�ڸ������У�����CNC����ģʽ2״̬���������ʱ���ڸ÷�����Ϊ100000
    function need_time=guosai_time_loss(RGV_location,M,t_shift)
        need_time=zeros(1,8);
        for gtl=1:8
            if M(gtl)==2
                need_time(gtl)=100000;
            else
                distance=guosai_calculate_distance(RGV_location,gtl);
                if distance==1
                    need_time(gtl)=t_shift(1);
                else if distance==2
                        need_time(gtl)=t_shift(2);
                    else if distance==3
                            need_time(gtl)=t_shift(3);
                        end
                    end
                end
            end
        end
        %����һ��������������RGV��CNC֮��ľ���
        function distance=guosai_calculate_distance(a,b)
            if (a>=1&&a<=2&&b>=1&&b<=2)||(a>=3&&a<=4&&b>=3&&b<=4)||(a>=5&&a<=6&&b>=5&&b<=6)||(a>=7&&a<=8&&b>=7&&b<=8)
                distance=0;
            else if (a>=1&&a<=2&&b>=5&&b<=6)||(a>=3&&a<=4&&b>=7&&b<=8)||(a>=5&&a<=6&&b>=1&&b<=2)||(a>=7&&a<=8&&b>=3&&b<=4)
                    distance=2;
                else if (a>=1&&a<=2&&b>=7&&b<=8)||(a>=7&&a<=8&&b>=1&&b<=2)
                        distance=3;
                    else
                        distance=1;
                    end
                end
            end           
        end
     
    end

%% ״̬����Mģʽ�л�����
    function M1=guosai_mode_change(M,change_where)
        M1=M;
        mode_ber=M(change_where);
        if mode_ber==1
            M1(change_where)=2;
        %else if mode_ber==2
        %       M1(change_where)=3;
            else if mode_ber==3
                    M1(change_where)=2;
                end
        end
    end

%% CNC�����ϵ�ʱ��
    function time=guosai_time_up(change_where,t_up)
        if mod(change_where,2)==1
            t_updown=t_up(1);
        else
            t_updown=t_up(2);
        end
        time=t_updown;
    end

%% ���CNC�Ƿ��������״̬3�仯Ϊ״̬2
    function numss=guosai_num_of_finish(M,M1)
        numss=0;
        for gnof=1:8
            if M(gnof)~=M1(gnof)
                if M(gnof)==3&&M1(gnof)==2
                    numss=numss+1;
                end
            end
        end
    end

%% ����Ƿ��з��������CNC�����Ƿ���CNC��״̬Ϊ1��3��
    function is=guosai_is_need_action(M)
        is=0;
        for gina=1:8
            if M(gina)==1||M(gina)==3
                is=1;
                break; 
            end
        end
    end

%% ����RGV���ƶ�֮���M�����TM������еĸ��º���
    function [TM,M1]=guosai_time_shift_go(M,M1,shift_time,TM,t_product)
        for gtsg=1:8
            if M(gtsg)==2
                TM(gtsg)=TM(gtsg)+shift_time;
                if TM(gtsg)>=t_product
                    TM(gtsg)=TM(gtsg)-t_product;
                    M1(gtsg)=3;
                end
            end
        end
    end

%% ��û��RGV�����˶���CNC���ڣ������е�CNC��Ϊ2״̬
    function [TM,M1,wait_time]=guosai_wait(M,TM,t_product)
        TMM=ones(1,8)*t_product;
        %�ҵ���С�ĵȴ�ֵ
        M1=M;
        TMM=TMM-TM;
        wait_time=TMM(1);
        for gwi=1:8
            if TMM(gwi)<=wait_time
                wait_time=TMM(gwi);
            end
        end
        for gwj=1:8
            TM(gwj)=TM(gwj)+wait_time;
            if TM(gwj)>=t_product
                TM(gwj)=TM(gwj)-t_product;
                M1(gwj)=3;
            end
        end
    end
%% ��������
    function guosai_conduct_data(product_name,begin_time,end_time,path)
        len=max(max(length(product_name),length(begin_time)),length(end_time));
        A=ones(len,3).*10010.10086;
        A(1:length(product_name),1)=product_name;
        A(1:length(begin_time),2)=begin_time;
        A(1:length(end_time),3)=end_time;
        A(:,2)=A(:,2)/3600;A(:,3)=A(:,3)/3600;
        for gcdi=1:len
            for gcdj=1:3
                if A(gcdi,gcdj)==10010.10086/3600
                    A(gcdi,gcdj)=NaN;
                end
            end
        end
        xlswrite(path,A);
    end

end