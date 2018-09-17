function [nums,product_biaohao1,product_biaohao2]=guosai_model_2(group)
%% 参数说明
%t_shift：向量，表示移动i个单位所需时间
%t_product:表示生产所需时间
%t_ud


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
%储存加工编号
product_biaohao1=[];
product_biaohao2=[];
%储存上下料的开始时间
begin1_time=[];
end1_time=[];
begin2_time=[];
end2_time=[];

%初始化状态矩阵M的模式(1状态和4状态为初始化状态)
M=ones(1,8);
M=guosai_initialize_M(t_product);
M_initialize=M;
M=[4 1 4 1 4 4 4 1];
%初始化MRGV的状态以描述RGV所能进行的工作：
%0状态：空无一物的状态
%1状态：手中含有第一道工序的成品，此时只能去找第二道工序的CNC
%该状态目前不予使用。2状态：手中含有第二道工序的成品，必须在进行清洗操作之后才能转化为状态0
MRGV=0;

%% 进入生产过程
while cost_time<=all_time
    %在本次轮回中进行的初始化
    clean_time=0;
    wait_time=0;
    per_cost=0;
    numss=0;
    %需要动作的话
    if guosai_is_need_action(M,MRGV)
        %计算RGV到每一个可行点的时间
        need_time=guosai_time_loss(RGV_location,M,MRGV,t_shift);
        %移动RGV并更新
        [M1,M1RGV,change_where,shift_time]=guosai_decision_tanxin(need_time,M,MRGV,t_up);
        
        %求取需要导出的数据
        if M(change_where)==1
            product_biaohao1=[product_biaohao1,change_where];
            begin1_time=[begin1_time,need_time(change_where)+cost_time];
        else if M(change_where)==3
                product_biaohao1=[product_biaohao1,change_where];
                begin1_time=[begin1_time,need_time(change_where)+cost_time];
                end1_time=[end1_time,need_time(change_where)+cost_time];
            else if M(change_where)==4
                    product_biaohao2=[product_biaohao2,change_where];
                    begin2_time=[begin2_time,need_time(change_where)+cost_time];
                else if M(change_where)==6
                        end2_time=[end2_time,need_time(change_where)+cost_time];
                        if MRGV==1
                            product_biaohao2=[product_biaohao2,change_where];
                            begin2_time=[begin2_time,need_time(change_where)+cost_time];
                        end
                    end
                end
            end
        end
       
        %计算清洗时间（如果该动作之后会存在的话）
        if M(change_where)==6
            clean_time=t_clean;
            numss=1;
        end
        shift_time=shift_time+clean_time;%总的时间
        %计算在RGV移动加料时间内状态为2或者5的CNC的时间流逝
        [TM,M1]=guosai_time_time_go(M,M1,shift_time,TM,t_product);
        
        %当前无任务，RGV处于等待状态
    else
        [TM,M1,wait_time]=guosai_wait(M,TM,t_product);
    end
    %计算本次决策运动（或者等待）开始到结束（也就是下一次决策或等待开始之前）的时间
    per_cost=wait_time+shift_time;
    
    %迭代更新
    RGV_location=change_where;
    M=M1;
    MRGV=M1RGV;
    cost_time=cost_time+per_cost;
    nums=nums+numss;

end

%将数据导出至EXCEL表格
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
xlswrite('D:\desktop\2nd_model_end.xls',M_conduct);

%% 定义确定负责哪一道工序的CNC的数量和分布位置情况
    function A=guosai_initialize_M(t_product)  %这里使用的是比例系数和随机位置法，并不具有权威性。
        alpha=t_product(1)/(t_product(1)+t_product(2));
        num1=round(alpha*8);
        num2=8-num1;
        A=ones(1,8);
        indexx=ceil(8*rand(1,num2)); %向上取整
        for giM=1:length(indexx)
            A(indexx(giM))=4;
        end
    end
    
%% 判断RGV在当前情况下是否需要动作
    function is=guosai_is_need_action(M,MRGV)
        is=0;
        for gina=1:length(M)
            if MRGV==0&&(M(gina)==1||M(gina)==3||M(gina)==6)
                is=1;
                break;
            else if MRGV==1&&(M(gina)==4||M(gina)==6)
                    is=1;
                    break;
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

%% 调度策略一：贪心算法。即每次都会选择花费时间最小的那一项操作进行。
    function [M1,M1RGV,change_where,shift_time]=guosai_decision_tanxin(need_time,M,MRGV,t_up)
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

%% 计算RGV无法进行动作时的等待时间
    function [TM,M1,wait_time]=guosai_wait(M,TM,t_product)
        TM_full=ones(1,8).*100000;
        for gwi=1:length(M)
            if M(gwi)==2
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
        %时间流逝
        for gwk=1:length(M)
            TM(gwk)=TM(gwk)+wait_time;
            if M(gwk)==2&&TM(gwk)>=t_product(1)
                M1(gwk)=3;
                TM(gwk)=TM(gwk)-t_product(1);
            else if M(gwk)==5&&TM(gwk)>=t_product(2)
                    M1(gwk)=6;
                    TM(gwk)=TM(gwk)-t_product(2);
                end
            end
        end
       
    end
        
end