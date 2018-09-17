function nums=guosai_model_1_monituihuofa(group)
%% ����˵��

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

% %��ʼ��״̬����ģʽ����Ϊ��ʼ��״̬��
% M=ones(1,8);
% %��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
% need_time=zeros(1,8);
% %��ʼ������ʱ��
% all_time=8*3600;
% cost_time=0;
% %��ʼ��RGV��λ��
% RGV_location=1;
% %��ʼ���ܹ����������������
% nums=0;
% %��ʼ������2״̬��CNC�ļӹ��ۼ�ʱ��
% TM=zeros(1,8);

% %����ӹ����
% product_number=[];
% %���������ϵĿ�ʼʱ��
% begin_time=[];
% end_time=[];

%��ʼ��ģ���˻�Ĳ���ֵ
T0=120;%�����ʼ�¶�
T_end=1;%���彵��֮����¶�
i=1;%������������
alpha=100;%������̶ܳ�ϵ��
zelta=0.99;%����˥��ϵ��
t=T0;
%����̰���㷨����һ����ŵĳ�ʼ��
[decision_vector,nums]=guosai_model_1_initial(t_shift,t_up,t_clean,t_product);
decision_vector
nums
%% ��ʼģ���˻�
while t>T_end
        
    %�����Ŷ����¶�Խ�ߣ��Ŷ�Խ��
    %num_of_epsinong=round((1-exp(-t*T_end/T0))*length(decision_vector));
    num_of_epsinong=1;
    where_index=ceil(rand(1,num_of_epsinong).*length(decision_vector));%��ȡ���ݽڵ������
    
    %ִ�ж�λ��ݳ�������µĽ�
    [decision_vector_new,nums_new]=guosai_goto_new(decision_vector,where_index,t_shift,t_product,t_clean,t_up);
    %�ж��µĽ��Ƿ��㹻����
    if nums_new>nums
        decision_vector=decision_vector_new;
        nums=nums_new;
    else if exp(alpha*(nums_new-nums)*t/T0)>rand(1)
            decision_vector=decision_vector_new;
            nums=nums_new;
        end
    end
    
    t=t*zelta;
    %����ѭ���ķ�ʽ�������¶���ȫ����֮�⣬�����õ���������������������Ȼ��������
    if nums>=400
        wordd='�㹻��'
        break;
    end
    
end
%�������Ž��·�����õ�����Ҫ��ÿ���������Ϣ�������䵼����Excel����С�
guosai_gogogo(decision_vector,t_shift,t_product,t_clean,t_up);

%% =========================================�Ӻ�����=======================================================

%% ����һ����ŵĳ�ʼ��ĺ���
    function [decision_vector,nums]=guosai_model_1_initial(t_shift,t_up,t_clean,t_product)
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
        decision_vector=[];
        %��ʼ������2״̬��CNC�ļӹ��ۼ�ʱ��
        TM=zeros(1,8);
        
        while cost_time<all_time
            numss=0;
            [M1,TM,change_where,per_cost]=guosai_decision_tanxin(M,RGV_location,TM,t_shift,t_up,t_clean,t_product);
            numss=guosai_num_of_finish(M,M1);
            %��������
            RGV_location=change_where;
            decision_vector=[decision_vector,change_where];
            nums=numss+nums;
            cost_time=cost_time+per_cost;
            M=M1;
        
        end
    end

%% ���ݵ���ǰ�Ľ���Ŷ������½�ĺ���
    function [decision_vector_new,nums_new]=guosai_goto_new(decision_vector,where,t_shift,t_product,t_clean,t_up)
        %��ʼ��CNC״̬����ģʽ����Ϊ��ʼ��״̬��
        M=ones(1,8);
        %��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
        RGV_location=1;
        need_time=zeros(1,8);
        %��ʼ��Լ��
        all_time=8*3600;
        cost_time=0;
        %��ʼ�����
        nums_new=0;
        decision_vector_new=[];
        %��ʼ������2״̬��CNC�ļӹ��ۼ�ʱ��
        TM=zeros(1,8);
        %while cost_time<all_time
        
        %����where�д�ŵ�ÿһ��Ԫ�ض�����Ҫ��ʼ�����Ŷ���λ��
        for ggni=1:length(where)
            if ggni==1 %��һ���Ŷ�Ϊ1�����һ���Ŷ�֮ǰ�����ж���Ϊ����ԭ���ľ�����������
                for ggnj=1:(where(ggni)-1)
                    decision_vector_new=decision_vector(1:(where(ggni)-1));
                    [M1,TM,this_cost_time]=guosai_go(M,RGV_location,decision_vector(ggnj),TM,t_shift,t_product,t_clean,t_up);
                    numss=guosai_num_of_finish(M,M1);
                    
                    %��������
                    RGV_location=decision_vector(ggnj);
                    cost_time=cost_time+this_cost_time;
                    M=M1;
                    nums_new=numss+nums_new;
                end
                %����Ҫ��ʼ����������ߣ�Ҳ���������п��еĵ�������֮��ѡ���Ǿֲ����ŵľ��ߡ�
                if where(ggni)-1==0
                    location=1;
                else
                    location=decision_vector(where(ggni)-1);
                end
                
                [M1,TM,change_where1,cost_time_emm]=guosai_decision_no_tanxin(M,location,TM,t_shift,t_up,t_clean,t_product);
                decision_vector_new=[decision_vector_new,change_where1];
                numss=guosai_num_of_finish(M,M1);
                nums_new=nums_new+numss;
                M=M1;
                cost_time=cost_time_emm+cost_time;
                if cost_time>=all_time
                    break;
                end
            end
        end
        %����������ÿһ�����Ǿֲ����ŵĽ��м���
        while cost_time<all_time
            %����̰�Ĳ��Ծ���
            [M1,TM,change_where,per_cost]=guosai_decision_tanxin(M,decision_vector_new(length(decision_vector_new)),TM,t_shift,t_up,t_clean,t_product);
            numss=guosai_num_of_finish(M,M1);
            %���е���֮��ĸ���
            nums_new=nums_new+numss;
            decision_vector_new=[decision_vector_new,change_where];
            M=M1;
            cost_time=cost_time+per_cost;
        end
        
    end

%% ����ִ��decision�еĲ������Ӷ��������ĵ�ʱ���Լ�CNC��״̬���и���
    function [M1,TM,this_cost_time]=guosai_go(M,lo1,lo2,TM,t_shift,t_product,t_clean,t_up)
        %��ʼ��
        shift_time=0;
        clean_time=0;
        wait_time=0;
        %�ж�RGV�ڽ����˶��Ĺ������Ƿ�����˵ȴ����������˵ȴ�����Ҫ����ʱ������һֱ��Ŀ��λ�õ�״ֵ̬Ϊ3
        if M(lo2)==2
            [TM,M,wait_time]=guosai_wait(TM,M,lo2,t_product);
        end
        distance=guosai_calculate_distance(lo1,lo2);
        if distance==1
            shift_time=t_shift(1);
        else if distance==2
                shift_time=t_shift(2);
            else if distance==3
                    shift_time=t_shift(3);
                end
            end
        end
        
        %�ж��Ƿ��������ϴ
        if M(lo2)==3
            clean_time=t_clean;
        end
        action_time=shift_time+guosai_time_up(lo2,t_up);
        %��״̬����ͼӹ����Ⱦ�����и���
        M1=guosai_mode_change(M,lo2);
        [TM,M1]=guosai_time_shift_go(M,M1,action_time+clean_time,TM,t_product);
        
        %��������ʱ��
        this_cost_time=action_time+clean_time+wait_time;
        
    end
         
%% ����һ������������������decision֮��ľ���
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
           
%% ��Ŀ��״̬Ϊ2״̬���ȴ���Ϊ3״̬
    function [TM,M1,wait_time]=guosai_wait(TM,M,location,t_product)
        wait_time=t_product-TM(location);
        M1=M;
        for gwj=1:8
            TM(gwj)=TM(gwj)+wait_time;
            if TM(gwj)>=t_product
                TM(gwj)=TM(gwj)-t_product;
                M1(gwj)=3;
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
            if M(gnof)~=1
                numss=1;
            end
        end
    end

%% ��̰�ľ��ߺ���
    function [M1,TM,change_where,cost_time_emm]=guosai_decision_no_tanxin(M,RGV_location,TM,t_shift,t_up,t_clean,t_product)
        wait_time=0;
        clean_time=0;
        %����Ϊ2״̬������Ҫ�ȴ�һ��ʱ��
        if ~guosai_is_need_action(M)
            [TM,M,wait_time]=guosai_M_wait(M,TM,t_product);
        end
        
        need_time=guosai_time_loss(RGV_location,M,t_shift);
        index_chosed=[];%����ɹ�ѡ�������CNC���
        for gdnt=1:length(need_time)
            if need_time(gdnt)~=100000
                index_chosed=[index_chosed,gdnt];
            end
        end
        change_where=ceil(rand(1)*length(index_chosed));
        %�鿴�Ƿ���Ҫ��ϴ
        if M(change_where)==3
            clean_time=t_clean;
        end
        %ģʽת����ʹʱ������
        M1=guosai_mode_change(M,change_where);
        shift_time=need_time(change_where);
        action_time=shift_time+guosai_time_up(change_where,t_up);
        [TM,M1]=guosai_time_shift_go(M,M1,action_time+clean_time,TM,t_product);
        
        cost_time_emm=action_time+wait_time+clean_time;    
        
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
    end

%% ��û��RGV�����˶���CNC���ڣ������е�CNC��Ϊ2״̬
    function [TM,M1,wait_time]=guosai_M_wait(M,TM,t_product)
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

%% ���оֲ����ž��ߣ�̰�ľ��ߣ�
    function [M1,TM,change_where,per_cost]=guosai_decision_tanxin(M,RGV_location,TM,t_shift,t_up,t_clean,t_product)
        %��ʼ��
        wait_time=0;
        clean_time=0;
        %����Ϊ2״̬������Ҫ�ȴ�һ��ʱ��
        if ~guosai_is_need_action(M)
            [TM,M,wait_time]=guosai_M_wait(M,TM,t_product);
        end
        
        need_time=guosai_time_loss(RGV_location,M,t_shift);
        decision_index=1;
        for gdt=1:8
            if (need_time(gdt)+guosai_time_up(gdt,t_up))<=(need_time(decision_index)+guosai_time_up(decision_index,t_up))
                decision_index=gdt;
            end
        end
        change_where=decision_index;
        %�鿴�Ƿ���Ҫ��ϴ
        if M(change_where)==3
            clean_time=t_clean;
        end
        %ģʽת����ʹʱ������
        M1=guosai_mode_change(M,change_where);
        shift_time=need_time(change_where);
        action_time=shift_time+guosai_time_up(change_where,t_up);
        [TM,M1]=guosai_time_shift_go(M,M1,action_time+clean_time,TM,t_product);
        
        per_cost=action_time+wait_time+clean_time;    
        
    end

%% ִ�к����������ݸ�����RGV��decsion_vectorִ�еõ�ÿһ���������ϸ�����Լ����е��������������֮������
    function guosai_gogogo(decision_vector,t_shift,t_product,t_clean,t_up)
        %��ʼ��״̬����ģʽ����Ϊ��ʼ��״̬��
        M=ones(1,8);
        %��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
        need_time=zeros(1,8);
        %��ʼ������ʱ��
        all_time=8*3600;
        cost_time=0;
        %��ʼ��RGV��λ��
        RGV_location=1;
        %��ʼ������2״̬��CNC�ļӹ��ۼ�ʱ��
        TM=zeros(1,8);
        
        %����ӹ����
        product_number=[];
        %���������ϵĿ�ʼʱ��
        begin_time=[];
        end_time=[];
        
        for gg=1:length(decision_vector)
            [M1,TM,this_cost_time]=guosai_go(M,RGV_location,decision_vector(gg),TM,t_shift,t_product,t_clean,t_up);
            
            change_where=decision_vector(gg);
            %��ȡ�����ϵ�ʱ��
            distance=guosai_calculate_distance(RGV_location,change_where);
            if distance==0
                shift_time=0;
            else
                shift_time=t_shift(distance);
            end
            % % ��ȡ�ӹ���CNC��ţ��Լ������ϵĿ�ʼʱ��
            if M(change_where)==1%�����ʼ���׶Σ�������
                product_number=[product_number,change_where];
                begin_time=[begin_time,(shift_time+cost_time)];%guosai_time_up(change_where,t_up)+
            else  %������������״̬3�����Թ�ȥ���Ͻ������ϣ���δ�����Ͻ�������
                product_number=[product_number,change_where];
                end_time=[end_time,(shift_time+cost_time)];%(guosai_time_up(change_where,t_up)+
                begin_time=[begin_time,(shift_time+cost_time)];%(guosai_time_up(change_where,t_up)+
            end
            
            %��������
            M=M1;
            cost_time=cost_time+this_cost_time;
            RGV_location=change_where;
        end
            
        %����ȡ�����ݵ�����EXCEL
        len_vec=[length(product_number),length(begin_time),length(end_time)];
        len=max(len_vec);
        AAA=ones(len,length(len_vec)).*20163933;
        AAA(1:len_vec(1),1)=product_number;
        AAA(1:len_vec(2),2)=begin_time;
        AAA(1:len_vec(3),3)=end_time;
        AAA(:,2:3)=AAA(:,2:3)/3600;
        
        for ggi=1:len
            for ggj=1:3
                if AAA(ggi,ggj)==20163933/3600
                    AAA(ggi,ggj)=NaN;
                end
            end
        end
        
        xlswrite('D:\desktop\ģ���˻�1.xls',AAA);
    
    end

end