function [nums,product_biaohao1,product_biaohao2]=guosai_model_2(group)
%% ����˵��
%t_shift����������ʾ�ƶ�i����λ����ʱ��
%t_product:��ʾ��������ʱ��
%t_ud


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
%����ӹ����
product_biaohao1=[];
product_biaohao2=[];
%���������ϵĿ�ʼʱ��
begin1_time=[];
end1_time=[];
begin2_time=[];
end2_time=[];

%��ʼ��״̬����M��ģʽ(1״̬��4״̬Ϊ��ʼ��״̬)
M=ones(1,8);
M=guosai_initialize_M(t_product);
M_initialize=M;
M=[4 1 4 1 4 4 4 1];
%��ʼ��MRGV��״̬������RGV���ܽ��еĹ�����
%0״̬������һ���״̬
%1״̬�����к��е�һ������ĳ�Ʒ����ʱֻ��ȥ�ҵڶ��������CNC
%��״̬Ŀǰ����ʹ�á�2״̬�����к��еڶ�������ĳ�Ʒ�������ڽ�����ϴ����֮�����ת��Ϊ״̬0
MRGV=0;

%% ������������
while cost_time<=all_time
    %�ڱ����ֻ��н��еĳ�ʼ��
    clean_time=0;
    wait_time=0;
    per_cost=0;
    numss=0;
    %��Ҫ�����Ļ�
    if guosai_is_need_action(M,MRGV)
        %����RGV��ÿһ�����е��ʱ��
        need_time=guosai_time_loss(RGV_location,M,MRGV,t_shift);
        %�ƶ�RGV������
        [M1,M1RGV,change_where,shift_time]=guosai_decision_tanxin(need_time,M,MRGV,t_up);
        
        %��ȡ��Ҫ����������
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
       
        %������ϴʱ�䣨����ö���֮�����ڵĻ���
        if M(change_where)==6
            clean_time=t_clean;
            numss=1;
        end
        shift_time=shift_time+clean_time;%�ܵ�ʱ��
        %������RGV�ƶ�����ʱ����״̬Ϊ2����5��CNC��ʱ������
        [TM,M1]=guosai_time_time_go(M,M1,shift_time,TM,t_product);
        
        %��ǰ������RGV���ڵȴ�״̬
    else
        [TM,M1,wait_time]=guosai_wait(M,TM,t_product);
    end
    %���㱾�ξ����˶������ߵȴ�����ʼ��������Ҳ������һ�ξ��߻�ȴ���ʼ֮ǰ����ʱ��
    per_cost=wait_time+shift_time;
    
    %��������
    RGV_location=change_where;
    M=M1;
    MRGV=M1RGV;
    cost_time=cost_time+per_cost;
    nums=nums+numss;

end

%�����ݵ�����EXCEL���
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

%% ����ȷ��������һ�������CNC�������ͷֲ�λ�����
    function A=guosai_initialize_M(t_product)  %����ʹ�õ��Ǳ���ϵ�������λ�÷�����������Ȩ���ԡ�
        alpha=t_product(1)/(t_product(1)+t_product(2));
        num1=round(alpha*8);
        num2=8-num1;
        A=ones(1,8);
        indexx=ceil(8*rand(1,num2)); %����ȡ��
        for giM=1:length(indexx)
            A(indexx(giM))=4;
        end
    end
    
%% �ж�RGV�ڵ�ǰ������Ƿ���Ҫ����
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

%% ���Ȳ���һ��̰���㷨����ÿ�ζ���ѡ�񻨷�ʱ����С����һ��������С�
    function [M1,M1RGV,change_where,shift_time]=guosai_decision_tanxin(need_time,M,MRGV,t_up)
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

%% ����RGV�޷����ж���ʱ�ĵȴ�ʱ��
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
        %ѡȡ��С�ȴ�ʱ��
        TM_res=TM_full-TM;
        wait_time=TM_res(1);
        for gwj=1:length(M)
            if TM_res(gwj)<=wait_time
                wait_time=TM_res(gwj);
            end
        end
        M1=M;
        %ʱ������
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