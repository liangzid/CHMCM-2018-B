function [nums_new,M_initialize]=guosai_model_2_monituihuofa(group)
%% 参数说明

%% 初始化
%对于运送时间的参数设定
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

%初始化状态矩阵的解的分布
M=ones(1,8);
M=guosai_initialize_M(t_product);
M_initialize=M;
if group==1
    M_initialize=[1 4 1 4 1 4 1 4];
end

%初始化模拟退火的参数值
T0=240;%定义初始温度
T_end=1;%定义降温之后的温度
i=1;%迭代次数定义
alpha=100;%定义接受程度系数
zelta=0.99;%定义衰减系数
t=T0;
%利用贪心算法产生一组较优的初始解
[decision_vector,nums]=guosai_model_2_initial(M_initialize,t_shift,t_up,t_clean,t_product);
nums
%% 开始模拟退火
while t>T_end
        
    %计算扰动，温度越高，扰动越大
    %num_of_epsinong=round((1-exp(-t*T_end/T0))*length(decision_vector));
    num_of_epsinong=1;
    where_index=ceil(rand(1,num_of_epsinong).*length(decision_vector));%求取回溯节点的索引
    
    %执行多次回溯程序产生新的解
    [decision_vector_new,nums_new]=guosai_goto_new(M_initialize,decision_vector,where_index,t_shift,t_product,t_clean,t_up);
    %判断新的解是否足够优秀
    if nums_new>nums
        decision_vector=decision_vector_new;
        nums=nums_new;
    else if exp(alpha*(nums_new-nums)*t/T0)>rand(1)
            decision_vector=decision_vector_new;
            nums=nums_new;
        end
    end
    
    t=t*zelta;
    %跳出循环的方式，除了温度完全降低之外，如若得到的数量满足了条件，仍然可以跳出
    if nums>=370
        wordd='足够了'
        break;
    end
    
end

%运行最优解的路径，得到所需要的每个零件的信息，并将其导出到Excel表格中。
nums_new=guosai_gogogo(M_initialize,decision_vector,t_shift,t_product,t_clean,t_up);
nums_new
%% ======================================子函数区====================================================

%% 利用贪心算法产生初始解的函数
    function [decision_vector,nums]=guosai_model_2_initial(M_initialize,t_shift,t_up,t_clean,t_product)
        
        %初始化RGV距离每一个CNC运动的所需时间
        need_time=zeros(1,8);
        %初始化所需时间
        all_time=8*3600;
        cost_time=0;
        %初始化RGV的位置
        RGV_location=1;
        %初始化总共生产的零件的数量
        nums=0;
        %初始化处于2状态或5状态的CNC的加工累计时间
        TM=zeros(1,8);
        %初始化状态矩阵M的模式(1状态和4状态为初始化状态)
        gm2i_M=M_initialize;
        %初始化MRGV的状态以描述RGV所能进行的工作：
        %0状态：空无一物的状态
        %1状态：手中含有第一道工序的成品，此时只能去找第二道工序的CNC
        %该状态目前不予使用。2状态：手中含有第二道工序的成品，必须在进行清洗操作之后才能转化为状态0
        MRGV=0;
        decision_vector=[];
        
        while cost_time<all_time
            numss=0;
            [M1,M1RGV,TM,change_where,per_cost,numss]=guosai_decision_tanxin(gm2i_M,MRGV,RGV_location,TM,t_shift,t_up,t_clean,t_product);
            %numss=guosai_num_of_finish(M,M1RGV,M1,M1RGV);
            %迭代更新
            RGV_location=change_where;
            decision_vector=[decision_vector,change_where];
            nums=numss+nums;
            cost_time=cost_time+per_cost;
            gm2i_M=M1;
            MRGV=M1RGV;
        end
    end

%% 定义确定负责哪一道工序的CNC的数量和分布位置情况
    function A=guosai_initialize_M(t_product)  %这里使用的是比例系数和随机位置法，并不具有权威性。
        alphaa=t_product(1)/(t_product(1)+t_product(2));
        num1=round(alphaa*8);
        num2=8-num1;
        A=ones(1,8);
        indexx=ceil(8*rand(1,num2)); %向上取整
        for giM=1:length(indexx)
            A(indexx(giM))=4;
        end
    end

%% 进行局部最优决策（贪心决策）
    function [M1,M1RGV,TM,change_where,per_cost,numss]=guosai_decision_tanxin(M,MRGV,RGV_location,TM,t_shift,t_up,t_clean,t_product)        
        wait_time=0;
        clean_time=0;
        numss=0;
        if guosai_is_need_wait(M,MRGV)
            [TM,M,MRGV,wait_time]=guosai_wait(M,MRGV,TM,t_product);
        end 
     
        need_time=guosai_time_loss(RGV_location,M,MRGV,t_shift); 
        %首先对不匹配的项进行删除（增大其需要的时间值）
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
        %找到匹配的最小的项,这里需要结合上下料时间
        change_where=1;
        shift_time=need_time(1)+t_up(1);
        for gdtxi=1:length(M)
            if (shift_time) >= (need_time(gdtxi)+guosai_time_up(gdtxi,t_up))
                shift_time=need_time(gdtxi)+guosai_time_up(gdtxi,t_up);
                change_where=gdtxi;
                
            end
        end
        %经过移动之后的更新
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
        
        %计算清洗时间
        if M(change_where)==6
            clean_time=t_clean;
            numss=1;
        end
        per_cost=shift_time+clean_time+wait_time;%总的时间
        %计算在RGV移动加料时间内状态为2或者5的CNC的时间流逝
        [TM,M1]=guosai_time_time_go(M,M1,shift_time+clean_time,TM,t_product);
        
    end

%% 判断RGV在当前情况下是否需要动作
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

%% 计算RGV无法进行动作时的等待时间
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
        %选取最小等待时间
        TM_res=TM_full-TM;
        wait_time=TM_res(1);
        for gwj=1:length(M)
            if TM_res(gwj)<=wait_time
                wait_time=TM_res(gwj);
            end
        end
        M1=M;
        M1RGV=MRGV;
        %时间流逝
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

%% 判断RGV移动到每一个可行点所需时间
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
            %对于无法匹配的项取一个极大的数，只对可以匹配的项进行计算。
            if is==0
                need_time(gtl)=100000;
            end 
        end
        %定义一个计算度量的函数
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

%% CNC上下料的时间
    function time=guosai_time_up(change_where,t_up)
        if mod(change_where,2)==1
            t_updown=t_up(1);
        else
            t_updown=t_up(2);
        end
        time=t_updown;
    end
    
%% 在RGV进行移动和上下料的时间中处于2状态或5状态的CNC的加工时间TM的更新以及状态矩阵M1的再更新
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

%% 定义根据已有决策向量和扰动添加位置得到新的决策向量和生产零件数量的函数
    function [decision_vector_new,nums_new]=guosai_goto_new(M,decision_vector,where_index,t_shift,t_product,t_clean,t_up)
        %注意这里的where_index就是一个数字就是一个数字就是一个数字
        
        %初始化RGV距离每一个CNC运动的所需时间
        need_time=zeros(1,8);
        %初始化所需时间
        all_time=8*3600;
        cost_time=0;
        %初始化RGV的位置
        RGV_location=1;
        %初始化总共生产的零件的数量
        nums_new=0;
        decision_vector_new=[];
        %初始化MRGV的状态以描述RGV所能进行的工作：
        %0状态：空无一物的状态
        %1状态：手中含有第一道工序的成品，此时只能去找第二道工序的CNC
        %该状态目前不予使用。2状态：手中含有第二道工序的成品，必须在进行清洗操作之后才能转化为状态0
        MRGV=0;
        TM=zeros(1,8);
        %改变索引之前的部分按照原来的流程运行
        decision_vector_new=decision_vector(1:(where_index-1));
        for ggn=1:(where_index-1)
            [M1,M1RGV,TM,this_cost_time,numss]=guosai_go(M,MRGV,RGV_location,decision_vector(ggn),TM,t_shift,t_product,t_clean,t_up);
            %迭代更新
            RGV_location=decision_vector(ggn);
            cost_time=cost_time+this_cost_time;
            M=M1;
            MRGV=M1RGV;
            nums_new=numss+nums_new;
        end
        %下面开始随机决策，即不是最优决策的调度策略
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
        
        %对进行了随机决策之后，采用局部最优的贪心算法对待以后的每一次决策
        while cost_time<all_time
            %进行贪心策略决策
            [M1,M1RGV,TM,change_where,per_cost,numss]=guosai_decision_tanxin(M,MRGV,decision_vector_new(length(decision_vector_new)),TM,t_shift,t_up,t_clean,t_product);
            
            %进行迭代之后的更新
            nums_new=nums_new+numss;
            decision_vector_new=[decision_vector_new,change_where];
            M=M1;
            MRGV=M1RGV;
            cost_time=cost_time+per_cost;
        end
        
    end

%% 按照给定的RGV运动策略求出RGV的运行时间、状态变化、是否加工完成零件等等
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
        
        %判断是否进行了清洗
        if M(lo2)==6
            clean_time=t_clean;
            numss=1;
        end
        action_time=shift_time+guosai_time_up(lo2,t_up);
        %对状态矩阵和加工进度矩阵进行更新
        [M1,M1RGV]=guosai_mode_change(M,MRGV,lo2);
        [TM,M1]=guosai_time_time_go(M,M1,action_time+clean_time,TM,t_product);
        
        %计算运行时间
        this_cost_time=action_time+clean_time+wait_time;
        
        
        
    
    end

%% 定义一个函数用来计算两个decision之间的距离
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


%% 对状态矩阵的状态进行更新的函数
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

%% 等待着目标位置的状态转换
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

%% 非贪婪的随机决策算法
    function [M1,M1RGV,TM,change_where1,cost_time_emm,numss]=guosai_decision_no_tanxin(M,MRGV,location,TM,t_shift,t_up,t_clean,t_product)
        wait_time=0;
        clean_time=0;
        numss=0;
        %若均为2状态，则需要等待一段时间
        if guosai_is_need_wait(M,MRGV)
            [TM,M,MRGV,wait_time]=guosai_wait(M,MRGV,TM,t_product);
        end
        need_time=guosai_time_loss(location,M,MRGV,t_shift);
        index_chosed=[];%储存可供选择的所有CNC编号
        
        for gdnt=1:length(need_time)
            if need_time(gdnt)~=100000
                index_chosed=[index_chosed,gdnt];
            end
        end
        %随机选择一个可用的仪器
        change_where1=ceil(rand(1)*length(index_chosed));
        %查看是否需要清洗
        if M(change_where1)==6
            clean_time=t_clean;
            numss=1;
        end
        
        %模式转换，使时间流逝
        [M1,M1RGV]=guosai_mode_change(M,MRGV,change_where1);
        shift_time=need_time(change_where1);
        action_time=shift_time+guosai_time_up(change_where1,t_up);
        [TM,M1]=guosai_time_time_go(M,M1,action_time+clean_time,TM,t_product);
        
        cost_time_emm=action_time+wait_time+clean_time; 
    
    end

%% 执行函数，将根据给出的RGV的decsion_vector执行得到每一个零件的详细数据以及所有的零件个数，并将之导出
    function nums=guosai_gogogo(M,decision_vector,t_shift,t_product,t_clean,t_up)
        
        nums=0;
        %初始化RGV距离每一个CNC运动的所需时间
        need_time=zeros(1,8);
        %初始化所需时间
        all_time=8*3600;
        cost_time=0;
        %初始化RGV的位置
        RGV_location=1;
        %初始化MRGV的状态以描述RGV所能进行的工作：
        %0状态：空无一物的状态
        %1状态：手中含有第一道工序的成品，此时只能去找第二道工序的CNC
        %该状态目前不予使用。2状态：手中含有第二道工序的成品，必须在进行清洗操作之后才能转化为状态0
        MRGV=0;
        TM=zeros(1,8);
        
        %储存加工编号
        product_biaohao1=[];
        product_biaohao2=[];
        %储存上下料的开始时间
        begin1_time=[];
        end1_time=[];
        begin2_time=[];
        end2_time=[];
        
        for gg=1:length(decision_vector)
            numss=0;
            
            [M1,M1RGV,TM,this_cost_time,numss]=guosai_go(M,MRGV,RGV_location,decision_vector(gg),TM,t_shift,t_product,t_clean,t_up);
            change_where=decision_vector(gg);
            
            %求取加工编号所需要用到的时间
            distance=guosai_calculate_distance(RGV_location,change_where);
            if distance==0
                shift_time=0;
            else
                shift_time=t_shift(distance);
            end
            
            %求取需要导出的数据
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
            
            %迭代更新
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
        xlswrite('D:\desktop\模拟退火法2.xls',M_conduct);
    end

end