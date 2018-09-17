function [nums,product_number,begin_time,end_time]=guosai_model_1(group)
%% 参数说明
%t_shift：向量，表示移动i个单位所需时间
%t_product:表示生产所需时间
%t_ud


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
%初始化处于2状态的CNC的加工累计时间
TM=zeros(1,8);

%储存加工编号
product_number=[];
%储存上下料的开始时间
begin_time=[];
end_time=[];

qwqw=1;
%% 开始进入循环，相当于描述整个过程
while cost_time<=all_time
    %定义在本次轮回中消耗的时间
    per_cost=0;
    wait_time=0;
    shift_time=0;
    change_where=RGV_location;
    numss=0;
    clean_time=0;
    shift_time=0;
    waste_time=0;
    %RGV动作
    if guosai_is_need_action(M)
        need_time=guosai_time_loss(RGV_location,M,t_shift);%计算所需时间-------------------------------------0
        %决策应该怎么走，并求得因此的迭代状态值矩阵、移动到的位置、移动所需的时间；该行代码运行完之后RGV便已经运动了
        [change_where,M1,shift_time]=guosai_decision_tanxin(M,need_time,t_up);%--------------------------1
        
        % % 求取加工的CNC编号，以及上下料的开始时间
        if M(change_where)==1%代表初始化阶段，即上料
            product_number=[product_number,change_where];
            begin_time=[begin_time,(need_time(change_where)+cost_time)];
        else  %即代表其属于状态3，即对过去的料进行下料，对未来的料进行上料
            product_number=[product_number,change_where];
            end_time=[end_time,(need_time(change_where)+cost_time)];
            begin_time=[begin_time,(need_time(change_where)+cost_time)];
        end
        
        %清洗的过程
        if M(change_where)==3
            clean_time=t_clean;
        end
        shift_time=shift_time+clean_time;
        %在运动时间上的处于2状态的CNC的加工时长
        [TM,M1]=guosai_time_shift_go(M,M1,shift_time,TM,t_product);
        %统计在进行运动之后是否会有零件完成了加工工作
        numss=guosai_num_of_finish(M,M1);%----------------------------------------------------------0
      
    %当下没有任务，RGV等待
    else
        [TM,M1,wait_time]=guosai_wait(M,TM,t_product);
    end
    %计算经过决策运动之后到下一次可以开始决策运动之前的总时间    
    per_cost=wait_time+shift_time;
    
    
    %进行迭代之后的更新
    RGV_location=change_where;
    M=M1;
    cost_time=cost_time+per_cost;
    nums=nums+numss;
    qwqw=qwqw+1;

end
%导出数据
guosai_conduct_data(product_number,begin_time,end_time,'D:\desktop\test.xls');



%% 贪心算法部分
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
        %定义一个函数用来计算RGV与CNC之间的距离
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
            if M(gnof)~=M1(gnof)
                if M(gnof)==3&&M1(gnof)==2
                    numss=numss+1;
                end
            end
        end
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

%% 当没有RGV可以运动的CNC存在，即所有的CNC均为2状态
    function [TM,M1,wait_time]=guosai_wait(M,TM,t_product)
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
%% 导出数据
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