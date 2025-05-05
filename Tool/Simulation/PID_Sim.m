close all
PID=[0.22,0.13,0;
    0.4,0.13,0;
    0.4,0.25,0;
    0.8,0.23,0.4;
    0.8,0.2,1;
    0.7,0.2,0.9];%初始化PID参数
for pid=1:1:6
ts=0.005;  %采样时间=0.005s
sys=tf(0.998,[0.021,1]);   %建立被控对象传递函数，即式4.1
dsys=c2d(sys,ts,'z');      %离散化
[num,den]=tfdata(dsys,'v');   %
e_1=0;      %前一时刻的偏差
Ee=0;       %累积偏差
u_1=0.0;    %前一时刻的控制量
y_1=0;       %前一时刻的输出
%PID参数
kp=PID(pid,1);    
ki=PID(pid,2);
kd=PID(pid,3);
u=zeros(1,1000);
time=zeros(1,1000);
for k=1:1:1000
    time(k)=k*ts;   %时间参数
    r(k)=1500;      %给定量
    y(k)=-1*den(2)*y_1+num(2)*u_1+num(1)*u(k);
    e(k)=r(k)-y(k);   %偏差
    u(k)=kp*e(k)+ki*Ee+kd*(e(k)-e_1);   
    Ee=Ee+e(k);    
    u_1=u(k);    
    y_1=y(k);    
    e_1=e(k);
end
subplot(2,3,pid);
p1=plot(time,r,'-.');xlim([0,1]);hold on;
p2=plot(time,y,'--');xlim([0,1]);
title(['Kp=',num2str(kp),' Ki=',num2str(ki),' Kd= ',num2str(kd)]);
hold on;
end
