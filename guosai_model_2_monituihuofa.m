function [nums_new,M_initialize]=guosai_model_2_monituihuofa(group)
%% ����˵��

%% ��ʼ��
%��������ʱ��Ĳ����趨
t_shift=[0 0 0];
t_product=[0,0];
t_up=[0 0];
t_clean=0;
if group==1
    t_shift=[20 33 46];
    t_product=[400,378];
    t_up=[28 31];
    t_clean=25;
    else if group==2
        t_shift=[23 41 59];
        t_product=[280,500];
        t_up=[30 35];
        t_clean=30;
        else if group==3
            t_shift=[18 32 46];
            t_product=[455,182];
            t_up=[27 32];
            t_clean=25;
            end
        end
end

%��ʼ��״̬����Ľ�ķֲ�
M=ones(1,8);
M=guosai_initialize_M(t_product);
M_initialize=M;
if group==1
    M_initialize=[1 4 1 4 1 4 1 4];
end

%��ʼ��ģ���˻�Ĳ���ֵ
T0=240;%�����ʼ�¶�
T_end=1;%���彵��֮����¶�
i=1;%������������
alpha=100;%������̶ܳ�ϵ��
zelta=0.99;%����˥��ϵ��
t=T0;
%����̰���㷨����һ����ŵĳ�ʼ��
[decision_vector,nums]=guosai_model_2_initial(M_initialize,t_shift,t_up,t_clean,t_product);
nums
%% ��ʼģ���˻�
while t>T_end
        
    %�����Ŷ����¶�Խ�ߣ��Ŷ�Խ��
    %num_of_epsinong=round((1-exp(-t*T_end/T0))*length(decision_vector));
    num_of_epsinong=1;
    where_index=ceil(rand(1,num_of_epsinong).*length(decision_vector));%��ȡ���ݽڵ������
    
    %ִ�ж�λ��ݳ�������µĽ�
    [decision_vector_new,nums_new]=guosai_goto_new(M_initialize,decision_vector,where_index,t_shift,t_product,t_clean,t_up);
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
    if nums>=370
        wordd='�㹻��'
        break;
    end
    
end

%�������Ž��·�����õ�����Ҫ��ÿ���������Ϣ�������䵼����Excel����С�
nums_new=guosai_gogogo(M_initialize,decision_vector,t_shift,t_product,t_clean,t_up);
nums_new
%% ======================================�Ӻ�����====================================================

%% ����̰���㷨������ʼ��ĺ���
    function [decision_vector,nums]=guosai_model_2_initial(M_initialize,t_shift,t_up,t_clean,t_product)
        
        %��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
        need_time=zeros(1,8);
        %��ʼ������ʱ��
        all_time=8*3600;
        cost_time=0;
        %��ʼ��RGV��λ��
        RGV_location=1;
        %��ʼ���ܹ����������������
        nums=0;
        %��ʼ������2״̬��5״̬��CNC�ļӹ��ۼ�ʱ��
        TM=zeros(1,8);
        %��ʼ��״̬����M��ģʽ(1״̬��4״̬Ϊ��ʼ��״̬)
        gm2i_M=M_initialize;
        %��ʼ��MRGV��״̬������RGV���ܽ��еĹ�����
        %0״̬������һ���״̬
        %1״̬�����к��е�һ������ĳ�Ʒ����ʱֻ��ȥ�ҵڶ��������CNC
        %��״̬Ŀǰ����ʹ�á�2״̬�����к��еڶ�������ĳ�Ʒ�������ڽ�����ϴ����֮�����ת��Ϊ״̬0
        MRGV=0;
        decision_vector=[];
        
        while cost_time<all_time
            numss=0;
            [M1,M1RGV,TM,change_where,per_cost,numss]=guosai_decision_tanxin(gm2i_M,MRGV,RGV_location,TM,t_shift,t_up,t_clean,t_product);
            %numss=guosai_num_of_finish(M,M1RGV,M1,M1RGV);
            %��������
            RGV_location=change_where;
            decision_vector=[decision_vector,change_where];
            nums=numss+nums;
            cost_time=cost_time+per_cost;
            gm2i_M=M1;
            MRGV=M1RGV;
        end
    end

%% ����ȷ��������һ�������CNC�������ͷֲ�λ�����
    function A=guosai_initialize_M(t_product)  %����ʹ�õ��Ǳ���ϵ�������λ�÷�����������Ȩ���ԡ�
        alphaa=t_product(1)/(t_product(1)+t_product(2));
        num1=round(alphaa*8);
        num2=8-num1;
        A=ones(1,8);
        indexx=ceil(8*rand(1,num2)); %����ȡ��
        for giM=1:length(indexx)
            A(indexx(giM))=4;
        end
    end

%% ���оֲ����ž��ߣ�̰�ľ��ߣ�
    function [M1,M1RGV,TM,change_where,per_cost,numss]=guosai_decision_tanxin(M,MRGV,RGV_location,TM,t_shift,t_up,t_clean,t_product)        
        wait_time=0;
        clean_time=0;
        numss=0;
        if guosai_is_need_wait(M,MRGV)
            [TM,M,MRGV,wait_time]=guosai_wait(M,MRGV,TM,t_product);
        end 
     
        need_time=guosai_time_loss(RGV_location,M,MRGV,t_shift); 
        %���ȶԲ�ƥ��������ɾ������������Ҫ��ʱ��ֵ��
        for gdtx=1:length(M)
            if MRGV==0
                if M(gdtx)==4
                    need_time(gdtx)=20000;
                end
            else if MRGV==1
                    if M(gdtx)~=4&&M(gdtx)~=6
                        need_time(gdtx)=20000;
                    end
                end
            end
        end
        %�ҵ�ƥ�����С����,������Ҫ���������ʱ��
        change_where=1;
        shift_time=need_time(1)+t_up(1);
        for gdtxi=1:length(M)
            if (shift_time) >= (need_time(gdtxi)+guosai_time_up(gdtxi,t_up))
                shift_time=need_time(gdtxi)+guosai_time_up(gdtxi,t_up);
                change_where=gdtxi;
                
            end
        end
        %�����ƶ�֮��ĸ���
        M1=M;
        M1RGV=MRGV;
        if MRGV==0
            if M(change_where)==1
                M1(change_where)=2;
            else if M(change_where)==3
                    M1(change_where)=2;
                    M1RGV=1;
                else if M(change_where)==6
                        M1(change_where)=4;
                    end
                end
            end
        else if MRGV==1
                if M(change_where)==4
                    M1(change_where)=5;
                    M1RGV=0;
                else if M(change_where)==6
                        M1(change_where)=5;
                        M1RGV=0;
                    end
                end
            end
        end
        
        %������ϴʱ��
        if M(change_where)==6
            clean_time=t_clean;
            numss=1;
        end
        per_cost=shift_time+clean_time+wait_time;%�ܵ�ʱ��
        %������RGV�ƶ�����ʱ����״̬Ϊ2����5��CNC��ʱ������
        [TM,M1]=guosai_time_time_go(M,M1,shift_time+clean_time,TM,t_product);
        
    end

%% �ж�RGV�ڵ�ǰ������Ƿ���Ҫ����
    function is=guosai_is_need_wait(M,MRGV)
        is=1;
        for gina=1:length(M)
            if MRGV==0&&(M(gina)==1||M(gina)==3||M(gina)==6)
                is=0;
                break;
            else if MRGV==1&&(M(gina)==4||M(gina)==6)
                    is=0;
                    break;
                end
            end
        end
    end

%% ����RGV�޷����ж���ʱ�ĵȴ�ʱ��
    function [TM,M1,M1RGV,wait_time]=guosai_wait(M,MRGV,TM,t_product)
        TM_full=ones(1,8).*100000;
        for gwi=1:length(M)
            if M(gwi)==2&&MRGV==0
                TM_full(gwi)=t_product(1);
            else if M(gwi)==5
                    TM_full(gwi)=t_product(2);
                end
            end
        end
        %ѡȡ��С�ȴ�ʱ��
        TM_res=TM_full-TM;
        wait_time=TM_res(1);
        for gwj=1:length(M)
            if TM_res(gwj)<=wait_time
                wait_time=TM_res(gwj);
            end
        end
        M1=M;
        M1RGV=MRGV;
        %ʱ������
        for gwk=1:length(M)
            TM(gwk)=TM(gwk)+wait_time;
            if M(gwk)==2&&TM(gwk)>=t_product(1)
                M1(gwk)=3;
                %M1RGV=1;
                TM(gwk)=TM(gwk)-t_product(1);
            else if M(gwk)==5&&TM(gwk)>=t_product(2)
                    M1(gwk)=6;
                    TM(gwk)=TM(gwk)-t_product(2);
                        
                    
                end
            end
        end
       
    end

%% �ж�RGV�ƶ���ÿһ�����е�����ʱ��
    function need_time=guosai_time_loss(RGV_location,M,MRGV,t_shift)
        need_time=zeros(1,8);
        
        for gtl=1:length(M)
            is=0;
            if ((MRGV==0)&&(M(gtl)==1||M(gtl)==3||M(gtl)==6))||((MRGV==1)&&(M(gtl)==4||M(gtl)==6))
                    is=1;
                    distance=guosai_calculate_distance(RGV_location,gtl);
                    if distance~=0
                        need_time(gtl)=t_shift(distance);
                    end
            end
            %�����޷�ƥ�����ȡһ�����������ֻ�Կ���ƥ�������м��㡣
            if is==0
                need_time(gtl)=100000;
            end 
        end
        %����һ����������ĺ���
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

%% CNC�����ϵ�ʱ��
    function time=guosai_time_up(change_where,t_up)
        if mod(change_where,2)==1
            t_updown=t_up(1);
        else
            t_updown=t_up(2);
        end
        time=t_updown;
    end
    
%% ��RGV�����ƶ��������ϵ�ʱ���д���2״̬��5״̬��CNC�ļӹ�ʱ��TM�ĸ����Լ�״̬����M1���ٸ���
    function [TM,M1]=guosai_time_time_go(M,M1,shift_time,TM,t_product)
        for gtto=1:length(M)
            TM(gtto)=TM(gtto)+shift_time;
            if M(gtto)==2
                if TM(gtto)>=t_product(1)
                    TM(gtto)=TM(gtto)-t_product(1);
                    M1(gtto)=3;
                end
            end
            if M(gtto)==5
                if TM(gtto)>=t_product(2)
                    TM(gtto)=TM(gtto)-t_product(2);
                    M1(gtto)=6;
                end
            end
        end
        
    end

%% ����������о����������Ŷ����λ�õõ��µľ���������������������ĺ���
    function [decision_vector_new,nums_new]=guosai_goto_new(M,decision_vector,where_index,t_shift,t_product,t_clean,t_up)
        %ע�������where_index����һ�����־���һ�����־���һ������
        
        %��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
        need_time=zeros(1,8);
        %��ʼ������ʱ��
        all_time=8*3600;
        cost_time=0;
        %��ʼ��RGV��λ��
        RGV_location=1;
        %��ʼ���ܹ����������������
        nums_new=0;
        decision_vector_new=[];
        %��ʼ��MRGV��״̬������RGV���ܽ��еĹ�����
        %0״̬������һ���״̬
        %1״̬�����к��е�һ������ĳ�Ʒ����ʱֻ��ȥ�ҵڶ��������CNC
        %��״̬Ŀǰ����ʹ�á�2״̬�����к��еڶ�������ĳ�Ʒ�������ڽ�����ϴ����֮�����ת��Ϊ״̬0
        MRGV=0;
        TM=zeros(1,8);
        %�ı�����֮ǰ�Ĳ��ְ���ԭ������������
        decision_vector_new=decision_vector(1:(where_index-1));
        for ggn=1:(where_index-1)
            [M1,M1RGV,TM,this_cost_time,numss]=guosai_go(M,MRGV,RGV_location,decision_vector(ggn),TM,t_shift,t_product,t_clean,t_up);
            %��������
            RGV_location=decision_vector(ggn);
            cost_time=cost_time+this_cost_time;
            M=M1;
            MRGV=M1RGV;
            nums_new=numss+nums_new;
        end
        %���濪ʼ������ߣ����������ž��ߵĵ��Ȳ���
        if where_index-1==0
            location=1;
        else
            location=decision_vector(where_index-1);
        end
        [M1,M1RGV,TM,change_where1,cost_time_emm,numss]=guosai_decision_no_tanxin(M,MRGV,location,TM,t_shift,t_up,t_clean,t_product);
        
        decision_vector_new=[decision_vector_new,change_where1];
        nums_new=nums_new+numss;
        M=M1;
        MRGV=M1RGV;
        cost_time=cost_time_emm+cost_time;
        
        %�Խ������������֮�󣬲��þֲ����ŵ�̰���㷨�Դ��Ժ��ÿһ�ξ���
        while cost_time<all_time
            %����̰�Ĳ��Ծ���
            [M1,M1RGV,TM,change_where,per_cost,numss]=guosai_decision_tanxin(M,MRGV,decision_vector_new(length(decision_vector_new)),TM,t_shift,t_up,t_clean,t_product);
            
            %���е���֮��ĸ���
            nums_new=nums_new+numss;
            decision_vector_new=[decision_vector_new,change_where];
            M=M1;
            MRGV=M1RGV;
            cost_time=cost_time+per_cost;
        end
        
    end

%% ���ո�����RGV�˶��������RGV������ʱ�䡢״̬�仯���Ƿ�ӹ��������ȵ�
    function [M1,M1RGV,TM,this_cost_time,numss]=guosai_go(M,MRGV,lo1,lo2,TM,t_shift,t_product,t_clean,t_up)
        wait_time=0;
        clean_time=0;
        shift_time=0;
        numss=0;
        need_time=guosai_time_loss(lo1,M,MRGV,t_shift);
        need_timee=need_time(lo2);
        
        if need_timee==100000
            [TM,M,wait_time]=guosai_wait_special(TM,lo2,M,t_product);
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
        if M(lo2)==6
            clean_time=t_clean;
            numss=1;
        end
        action_time=shift_time+guosai_time_up(lo2,t_up);
        %��״̬����ͼӹ����Ⱦ�����и���
        [M1,M1RGV]=guosai_mode_change(M,MRGV,lo2);
        [TM,M1]=guosai_time_time_go(M,M1,action_time+clean_time,TM,t_product);
        
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


%% ��״̬�����״̬���и��µĺ���
    function  [M1,M1RGV]=guosai_mode_change(M,MRGV,change_where)
        M1=M;
        M1RGV=MRGV;
        if MRGV==0
            if M(change_where)==1
                M1(change_where)=2;
            else if M(change_where)==3
                    M1(change_where)=2;
                    M1RGV=1;
                else if M(change_where)==6
                        M1(change_where)=4;
                    end
                end
            end
        else if MRGV==1
                if M(change_where)==4
                    M1(change_where)=5;
                    M1RGV=0;
                else if M(change_where)==6
                        M1(change_where)=5;
                        M1RGV=0;
                    end
                end
            end
        end
    
    end

%% �ȴ���Ŀ��λ�õ�״̬ת��
    function [TM,M1,wait_time]=guosai_wait_special(TM,location,M,t_product)
        product_time=0;
        M1=M;
        if M(location)==2
            product_time=t_product(1);
        else
            product_time=t_product(2);
        end
        wait_time=product_time-TM(location);
        for gws=1:length(TM)
            TM(gws)=TM(gws)+wait_time;
            if TM(gws)>=product_time
                TM(gws)=TM(gws)-product_time;
                if M(gws)==2
                    M1(gws)=3;
                else if M(gws)==5
                        M1(gws)=6;
                    end
                end
            end
        end    
    end

%% ��̰������������㷨
    function [M1,M1RGV,TM,change_where1,cost_time_emm,numss]=guosai_decision_no_tanxin(M,MRGV,location,TM,t_shift,t_up,t_clean,t_product)
        wait_time=0;
        clean_time=0;
        numss=0;
        %����Ϊ2״̬������Ҫ�ȴ�һ��ʱ��
        if guosai_is_need_wait(M,MRGV)
            [TM,M,MRGV,wait_time]=guosai_wait(M,MRGV,TM,t_product);
        end
        need_time=guosai_time_loss(location,M,MRGV,t_shift);
        index_chosed=[];%����ɹ�ѡ�������CNC���
        
        for gdnt=1:length(need_time)
            if need_time(gdnt)~=100000
                index_chosed=[index_chosed,gdnt];
            end
        end
        %���ѡ��һ�����õ�����
        change_where1=ceil(rand(1)*length(index_chosed));
        %�鿴�Ƿ���Ҫ��ϴ
        if M(change_where1)==6
            clean_time=t_clean;
            numss=1;
        end
        
        %ģʽת����ʹʱ������
        [M1,M1RGV]=guosai_mode_change(M,MRGV,change_where1);
        shift_time=need_time(change_where1);
        action_time=shift_time+guosai_time_up(change_where1,t_up);
        [TM,M1]=guosai_time_time_go(M,M1,action_time+clean_time,TM,t_product);
        
        cost_time_emm=action_time+wait_time+clean_time; 
    
    end

%% ִ�к����������ݸ�����RGV��decsion_vectorִ�еõ�ÿһ���������ϸ�����Լ����е��������������֮����
    function nums=guosai_gogogo(M,decision_vector,t_shift,t_product,t_clean,t_up)
        
        nums=0;
        %��ʼ��RGV����ÿһ��CNC�˶�������ʱ��
        need_time=zeros(1,8);
        %��ʼ������ʱ��
        all_time=8*3600;
        cost_time=0;
        %��ʼ��RGV��λ��
        RGV_location=1;
        %��ʼ��MRGV��״̬������RGV���ܽ��еĹ�����
        %0״̬������һ���״̬
        %1״̬�����к��е�һ������ĳ�Ʒ����ʱֻ��ȥ�ҵڶ��������CNC
        %��״̬Ŀǰ����ʹ�á�2״̬�����к��еڶ�������ĳ�Ʒ�������ڽ�����ϴ����֮�����ת��Ϊ״̬0
        MRGV=0;
        TM=zeros(1,8);
        
        %����ӹ����
        product_biaohao1=[];
        product_biaohao2=[];
        %���������ϵĿ�ʼʱ��
        begin1_time=[];
        end1_time=[];
        begin2_time=[];
        end2_time=[];
        
        for gg=1:length(decision_vector)
            numss=0;
            
            [M1,M1RGV,TM,this_cost_time,numss]=guosai_go(M,MRGV,RGV_location,decision_vector(gg),TM,t_shift,t_product,t_clean,t_up);
            change_where=decision_vector(gg);
            
            %��ȡ�ӹ��������Ҫ�õ���ʱ��
            distance=guosai_calculate_distance(RGV_location,change_where);
            if distance==0
                shift_time=0;
            else
                shift_time=t_shift(distance);
            end
            
            %��ȡ��Ҫ����������
            if M(change_where)==1
                product_biaohao1=[product_biaohao1,change_where];
                begin1_time=[begin1_time,shift_time+cost_time];
            else if M(change_where)==3
                    product_biaohao1=[product_biaohao1,change_where];
                    begin1_time=[begin1_time,shift_time+cost_time];
                    end1_time=[end1_time,shift_time+cost_time];
                else if M(change_where)==4
                        product_biaohao2=[product_biaohao2,change_where];
                        begin2_time=[begin2_time,shift_time+cost_time];
                    else if M(change_where)==6
                            end2_time=[end2_time,shift_time+cost_time];
                            if MRGV==1
                                product_biaohao2=[product_biaohao2,change_where];
                                begin2_time=[begin2_time,shift_time+cost_time];
                            end
                        end
                    end
                end
            end
            
            %��������
            RGV_location=change_where;
            M=M1;
            MRGV=M1RGV;
            cost_time=cost_time+this_cost_time;
            nums=nums+numss;
        end
        
        length_vector=[length(product_biaohao1),length(begin1_time),length(end1_time),length(product_biaohao2),length(begin2_time),length(end2_time) ];
        len=max(length_vector);
        M_conduct=ones(len,6).*20163933;
        M_conduct(1:length(product_biaohao1),1)=product_biaohao1;
        M_conduct(1:length(begin1_time),2)=begin1_time;
        M_conduct(1:length(end1_time),3)=end1_time;
        M_conduct(1:length(product_biaohao2),4)=product_biaohao2;
        M_conduct(1:length(begin2_time),5)=begin2_time;
        M_conduct(1:length(end2_time),6)=end2_time;
        M_conduct(:,2:3)=M_conduct(:,2:3)/3600;
        M_conduct(:,5:6)=M_conduct(:,5:6)/3600;
        for ii=1:len
            for jj=1:6
                if M_conduct(ii,jj)==20163933/3600||M_conduct(ii,jj)==20163933
                    M_conduct(ii,jj)=NaN;
                end
            end
        end
        xlswrite('D:\desktop\ģ���˻�2.xls',M_conduct);
    end

end