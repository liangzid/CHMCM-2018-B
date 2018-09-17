function nums=guosai_model_1_monituihuofa(group)
%% 参数说明

%% 初始化
%对于运送时间的参数设定
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

% %初始化状态矩阵模式（均为初始化状态）
% M=ones(1,8);
% %初始化RGV距离每一个CNC运动的所需时间
% need_time=zeros(1,8);
% %初始化所需时间
% all_time=8*3600;
% cost_time=0;
% %初始化RGV的位置
% RGV_location=1;
% %初始化总共生产的零件的数量
% nums=0;
% %初始化处于2状态的CNC的加工累计时间
% TM=zeros(1,8);

% %储存加工编号
% product_number=[];
% %储存上下料的开始时间
% begin_time=[];
% end_time=[];

%初始化模拟退火的参数值
T0=120;%定义初始温度
T_end=1;%定义降温之后的温度
i=1;%迭代次数定义
alpha=100;%定义接受程度系数
zelta=0.99;%定义衰减系数
t=T0;
%利用贪心算法产生一组较优的初始解
[decision_vector,nums]=guosai_model_1_initial(t_shift,t_up,t_clean,t_product);
decision_vector
nums
%% 开始模拟退火
while t>T_end
        
    %计算扰动，温度越高，扰动越大
    %num_of_epsinong=round((1-exp(-t*T_end/T0))*length(decision_vector));
    num_of_epsinong=1;
    where_index=ceil(rand(1,num_of_epsinong).*length(decision_vector));%求取回溯节点的索引
    
    %执行多次回溯程序产生新的解
    [decision_vector_new,nums_new]=guosai_goto_new(decision_vector,where_index,t_shift,t_product,t_clean,t_up);
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
    if nums>=400
        wordd='足够了'
        break;
    end
    
end
%运行最优解的路径，得到所需要的每个零件的信息，并将其导出到Excel表格中。
guosai_gogogo(decision_vector,t_shift,t_product,t_clean,t_up);

%% =========================================子函数区=======================================================

%% 产生一组较优的初始解的函数
    function [decision_vector,nums]=guosai_model_1_initial(t_shift,t_up,t_clean,t_product)
        %初始化状态矩阵模式（均为初始化状态）
        M=ones(1,8);
        %初始化RGV距离每一个CNC运动的所需时间
        need_time=zeros(1,8);
        %初始化所需时间
        all_time=8*3600;
        cost_time=0;
        %初始化RGV的位置
        RGV_location=1;
        %初始化总共生产的零件的数量
        nums=0;
        decision_vector=[];
        %初始化处于2状态的CNC的加工累计时间
        TM=zeros(1,8);
        
        while cost_time<all_time
            numss=0;
            [M1,TM,change_where,per_cost]=guosai_decision_tanxin(M,RGV_location,TM,t_shift,t_up,t_clean,t_product);
            numss=guosai_num_of_finish(M,M1);
            %迭代更新
            RGV_location=change_where;
            decision_vector=[decision_vector,change_where];
            nums=numss+nums;
            cost_time=cost_time+per_cost;
            M=M1;
        
        end
    end

%% 根据迭代前的解和扰动产生新解的函数
    function [decision_vector_new,nums_new]=guosai_goto_new(decision_vector,where,t_shift,t_product,t_clean,t_up)
        %初始化CNC状态矩阵模式（均为初始化状态）
        M=ones(1,8);
        %初始化RGV距离每一个CNC运动的所需时间
        RGV_location=1;
        need_time=zeros(1,8);
        %初始化约束
        all_time=8*3600;
        cost_time=0;
        %初始化输出
        nums_new=0;
        decision_vector_new=[];
        %初始化处于2状态的CNC的加工累计时间
        TM=zeros(1,8);
        %while cost_time<all_time
        
        %向量where中存放的每一个元素都是需要开始进行扰动的位置
        for ggni=1:length(where)
            if ggni==1 %第一个扰动为1，则第一个扰动之前的所有都视为按照原来的决策向量决策
                for ggnj=1:(where(ggni)-1)
                    decision_vector_new=decision_vector(1:(where(ggni)-1));
                    [M1,TM,this_cost_time]=guosai_go(M,RGV_location,decision_vector(ggnj),TM,t_shift,t_product,t_clean,t_up);
                    numss=guosai_num_of_finish(M,M1);
                    
                    %迭代更新
                    RGV_location=decision_vector(ggnj);
                    cost_time=cost_time+this_cost_time;
                    M=M1;
                    nums_new=numss+nums_new;
                end
                %下面要开始进行随机决策，也就是在所有可行的单步决策之中选择不是局部最优的决策。
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
        %接下来进行每一步都是局部最优的进行计算
        while cost_time<all_time
            %进行贪心策略决策
            [M1,TM,change_where,per_cost]=guosai_decision_tanxin(M,decision_vector_new(length(decision_vector_new)),TM,t_shift,t_up,t_clean,t_product);
            numss=guosai_num_of_finish(M,M1);
            %进行迭代之后的更新
            nums_new=nums_new+numss;
            decision_vector_new=[decision_vector_new,change_where];
            M=M1;
            cost_time=cost_time+per_cost;
        end
        
    end

%% 用来执行decision中的操作，从而对所消耗的时间以及CNC的状态进行更新
    function [M1,TM,this_cost_time]=guosai_go(M,lo1,lo2,TM,t_shift,t_product,t_clean,t_up)
        %初始化
        shift_time=0;
        clean_time=0;
        wait_time=0;
        %判断RGV在进行运动的过程中是否进行了等待，若进行了等待，需要进行时间流逝一直到目标位置的状态值为3
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
        
        %判断是否进行了清洗
        if M(lo2)==3
            clean_time=t_clean;
        end
        action_time=shift_time+guosai_time_up(lo2,t_up);
        %对状态矩阵和加工进度矩阵进行更新
        M1=guosai_mode_change(M,lo2);
        [TM,M1]=guosai_time_shift_go(M,M1,action_time+clean_time,TM,t_product);
        
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
           
%% 若目标状态为2状态，等待其为3状态
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

%% 状态矩阵M模式切换函数
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

%% 随着RGV的移动之后对M矩阵和TM矩阵进行的更新函数
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

%% CNC上下料的时间
    function time=guosai_time_up(change_where,t_up)
        if mod(change_where,2)==1
            t_updown=t_up(1);
        else
            t_updown=t_up(2);
        end
        time=t_updown;
    end

%% 检测CNC是否有零件从状态3变化为状态2
    function numss=guosai_num_of_finish(M,M1)
        numss=0;
        for gnof=1:8
            if M(gnof)~=1
                numss=1;
            end
        end
    end

%% 非贪心决策函数
    function [M1,TM,change_where,cost_time_emm]=guosai_decision_no_tanxin(M,RGV_location,TM,t_shift,t_up,t_clean,t_product)
        wait_time=0;
        clean_time=0;
        %若均为2状态，则需要等待一段时间
        if ~guosai_is_need_action(M)
            [TM,M,wait_time]=guosai_M_wait(M,TM,t_product);
        end
        
        need_time=guosai_time_loss(RGV_location,M,t_shift);
        index_chosed=[];%储存可供选择的所有CNC编号
        for gdnt=1:length(need_time)
            if need_time(gdnt)~=100000
                index_chosed=[index_chosed,gdnt];
            end
        end
        change_where=ceil(rand(1)*length(index_chosed));
        %查看是否需要清洗
        if M(change_where)==3
            clean_time=t_clean;
        end
        %模式转换，使时间流逝
        M1=guosai_mode_change(M,change_where);
        shift_time=need_time(change_where);
        action_time=shift_time+guosai_time_up(change_where,t_up);
        [TM,M1]=guosai_time_shift_go(M,M1,action_time+clean_time,TM,t_product);
        
        cost_time_emm=action_time+wait_time+clean_time;    
        
    end

%% 检测是否有发送请求的CNC（即是否有CNC的状态为1或3）
    function is=guosai_is_need_action(M)
        is=0;
        for gina=1:8
            if M(gina)==1||M(gina)==3
                is=1;
                break; 
            end
        end
    end

%% 计算任意两个点之间的花费
%在该问题中，如若CNC处于模式2状态，则所需的时间在该分量上为100000
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

%% 当没有RGV可以运动的CNC存在，即所有的CNC均为2状态
    function [TM,M1,wait_time]=guosai_M_wait(M,TM,t_product)
        TMM=ones(1,8)*t_product;
        %找到最小的等待值
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

%% 进行局部最优决策（贪心决策）
    function [M1,TM,change_where,per_cost]=guosai_decision_tanxin(M,RGV_location,TM,t_shift,t_up,t_clean,t_product)
        %初始化
        wait_time=0;
        clean_time=0;
        %若均为2状态，则需要等待一段时间
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
        %查看是否需要清洗
        if M(change_where)==3
            clean_time=t_clean;
        end
        %模式转换，使时间流逝
        M1=guosai_mode_change(M,change_where);
        shift_time=need_time(change_where);
        action_time=shift_time+guosai_time_up(change_where,t_up);
        [TM,M1]=guosai_time_shift_go(M,M1,action_time+clean_time,TM,t_product);
        
        per_cost=action_time+wait_time+clean_time;    
        
    end

%% 执行函数，将根据给出的RGV的decsion_vector执行得到每一个零件的详细数据以及所有的零件个数，并将之导出。
    function guosai_gogogo(decision_vector,t_shift,t_product,t_clean,t_up)
        %初始化状态矩阵模式（均为初始化状态）
        M=ones(1,8);
        %初始化RGV距离每一个CNC运动的所需时间
        need_time=zeros(1,8);
        %初始化所需时间
        all_time=8*3600;
        cost_time=0;
        %初始化RGV的位置
        RGV_location=1;
        %初始化处于2状态的CNC的加工累计时间
        TM=zeros(1,8);
        
        %储存加工编号
        product_number=[];
        %储存上下料的开始时间
        begin_time=[];
        end_time=[];
        
        for gg=1:length(decision_vector)
            [M1,TM,this_cost_time]=guosai_go(M,RGV_location,decision_vector(gg),TM,t_shift,t_product,t_clean,t_up);
            
            change_where=decision_vector(gg);
            %求取上下料的时间
            distance=guosai_calculate_distance(RGV_location,change_where);
            if distance==0
                shift_time=0;
            else
                shift_time=t_shift(distance);
            end
            % % 求取加工的CNC编号，以及上下料的开始时间
            if M(change_where)==1%代表初始化阶段，即上料
                product_number=[product_number,change_where];
                begin_time=[begin_time,(shift_time+cost_time)];%guosai_time_up(change_where,t_up)+
            else  %即代表其属于状态3，即对过去的料进行下料，对未来的料进行上料
                product_number=[product_number,change_where];
                end_time=[end_time,(shift_time+cost_time)];%(guosai_time_up(change_where,t_up)+
                begin_time=[begin_time,(shift_time+cost_time)];%(guosai_time_up(change_where,t_up)+
            end
            
            %迭代更新
            M=M1;
            cost_time=cost_time+this_cost_time;
            RGV_location=change_where;
        end
            
        %将获取的数据导出至EXCEL
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
        
        xlswrite('D:\desktop\模拟退火法1.xls',AAA);
    
    end

end