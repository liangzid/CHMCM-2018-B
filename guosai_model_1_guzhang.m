function guosai_model_1_guzhang(group)
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
t_error=[10 20];%���������������֮���ܹ��ָ����������ʱ�����ȷ������ȷ�硣
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
TM_error=zeros(2,8);%���崢��ʱ��ľ���TM_error(1,:)����ǰ���ŵ�ʱ�䣬TM_error(2,:)�����޲�ʱ�������

%����ӹ����
product_number=[];
%���������ϵĿ�ʼʱ��
begin_time=[];
end_time=[];

%�������ʱ�����ϱ��
product_error_number=[];
%�������ʱ��CNC���
CNC_error_number=[];
%������ϵĿ�ʼ����ʱ��
begin_time_error=[];
end_time_error=[];

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
            if M1(change_where)==-1
                product_error_number=[product_error_number,length(product_number)];
                CNC_error_number=[CNC_error_number,change_where];
                begin_time_error=[begin_time_error,need_time(change_where)+cost_time];
                time=rand(1)*600+600;
                end_time_error=[end_time_error,time+(need_time(change_where)+cost_time)];
                TM_error(2,change_where)=time;
            end
        else  %������������״̬3�����Թ�ȥ���Ͻ������ϣ���δ�����Ͻ�������
            product_number=[product_number,change_where];
            end_time=[end_time,(need_time(change_where)+cost_time)];
            begin_time=[begin_time,(need_time(change_where)+cost_time)];
            if M1(change_where)==-1
                product_error_number=[product_error_number,length(product_number)];
                CNC_error_number=[CNC_error_number,change_where];
                begin_time_error=[begin_time_error,need_time(change_where)+cost_time];
                time=rand(1)*600+600;
                end_time_error=[end_time_error,time+(need_time(change_where)+cost_time)];
                TM_error(2,change_where)=time;
            end
        end
        
        %��ϴ�Ĺ���
        if M(change_where)==3
            clean_time=t_clean;
        end
        shift_time=shift_time+clean_time;
        %���˶�ʱ���ϵĴ���2״̬��CNC�ļӹ�ʱ��
        [TM,TM_error,M1]=guosai_time_shift_go(M,M1,shift_time,TM,TM_error,t_product);
        %ͳ���ڽ����˶�֮���Ƿ�����������˼ӹ�����
        numss=guosai_num_of_finish(M,M1);%----------------------------------------------------------0
        %����û������RGV�ȴ�
    else
        [TM,TM_error,M1,wait_time]=guosai_wait(M,TM,TM_error,t_product);
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
guosai_conduct_data(product_number,begin_time,end_time,product_error_number,CNC_error_number,begin_time_error,end_time_error,'D:\desktop\���ǹ��ϵ����1.xls');



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
            else if M(gtl)==-1
                    need_time(gtl)=300000;
                else if M(gtl)==1||M(gtl)==3
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
            if rand(1)>=0.01
                M1(change_where)=2;
            else
                M1(change_where)=-1;
            end
            %else if mode_ber==2
            %       M1(change_where)=3;
        else if mode_ber==3
                if rand(1)>=0.01
                    M1(change_where)=2;
                else
                    M1(change_where)=-1;
                end
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
    function [TM,TM_error,M1]=guosai_time_shift_go(M,M1,shift_time,TM,TM_error,t_product)
        for gtsg=1:8
            if M(gtsg)==2
                TM(gtsg)=TM(gtsg)+shift_time;
                if TM(gtsg)>=t_product
                    TM(gtsg)=0;
                    M1(gtsg)=3;
                end
            else if M(gtsg)==-1
                    TM_error(1,gtsg)=TM_error(1,gtsg)+shift_time;
                    if TM_error(gtsg)>TM_error(2,gtsg)
                        TM_error(1,gtsg)=0;
                        TM_error(2,gtsg)=0;
                        M1(gtsg)=1;
                    end
                end
            end
        end
    end

%% ��û��RGV�����˶���CNC���ڣ������е�CNC��Ϊ2״̬��-1״̬
    function [TM,TM_error,M1,wait_time]=guosai_wait(M,TM,TM_error,t_product)
        TMM=ones(1,8).*t_product;
        %�ҵ���С�ĵȴ�ֵ
        M1=M;
        TMM=TMM-TM;
        wait_time=TMM(1);
        for gwi=1:8
            if TMM(gwi)<=wait_time
                wait_time=TMM(gwi);
            end
        end
        
        guzhang_wait=ones(1,8).*4000;
        for gwii=1:8
            if TM_error(2,gwii)>0
                guzhang_wait(gwii)=TM_error(2,gwii)-TM_error(1,gwii);
            end
        end
        wait_time=min(wait_time,min(guzhang_wait));
        
        for gwj=1:8
            TM(gwj)=TM(gwj)+wait_time;
            TM_error(1,gwj)=TM_error(1,gwj)+wait_time;
            if TM(gwj)>=t_product
                TM(gwj)=0;
                M1(gwj)=3;
            else
                TM_error(1,gwj)=0;
                TM_error(2,gwj)=0;
                M1(gwj)=1;
            end
            
        end
    end
%% ��������
    function guosai_conduct_data(product_number,begin_time,end_time,product_error_number,CNC_error_number,begin_time_error,end_time_error,path)
        len_vector=[length(product_number) length(begin_time) length(end_time) ];
        error_vector=[length(product_error_number) length(CNC_error_number) length(begin_time_error) length(end_time_error)];
        len=min(len_vector);
        len_len=length(len_vector);
        A=ones(len,len_len);
        A(:,1)=product_number(1:len);
        A(:,2)=begin_time(1:len);
        A(:,3)=end_time(1:len);
        A(:,2:3)=A(:,2:3)/3600;
        len2=min(error_vector);len2_len=length(error_vector);
        B=ones(len2,len2_len);
        B(:,1)=product_error_number(1:len2);
        B(:,2)=CNC_error_number(1:len2);
        B(:,3)=begin_time_error(1:len2);
        B(:,4)=end_time_error(1:len2);
        B(:,3:4)=B(:,3:4)/3600;
        
        xlswrite(path,A,'sheet1');
        xlswrite(path,B,'sheet2');
    end

end