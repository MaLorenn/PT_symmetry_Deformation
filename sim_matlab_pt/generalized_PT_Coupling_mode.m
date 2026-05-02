 clc; 
clear;
%clf(1);
%------------------------------------------------------------------
%Robust wireless power transfer using a nonlinear parity–time-symmetric circuit
%------------------------------------------------------------------
% 广义PT下的图
% 找到解决方案了，和大家分享一下，出现这样的情况是因为数据量过大，
% 直接生成eps文件或者从fig中导出都会出现上述问题，
% 直接在代码中添加print -depsc2 -painters filename就可以生成可编辑的eps
%------------------------------------------------------------------
%定义方程组如下，其中k为变量
 syms w g k
 w1=1;w2=1;r2=0.1;%定义发射端W1与接收端W2，偏离参数在这改
  eqns = (w-w1)*(w-w2)^2 + (w-w1)*r2^2-(w-w2).*k.^2 ; %方程3 
  wr=solve(eqns,w);  
 i=0;n=20; k_start=1e-3; k_end=0.2;%定义k的范围
 for k1=linspace(k_start, k_end, n)
 i=i+1;
 wj(1:3,i)=real(subs(wr, k, k1));
 end
 wj=single(sort(wj));
k=linspace(k_start, k_end, n); 
figure(1);hold on;grid on;

shift_start=-0.5;shift_end=0.5;
for a=shift_start:0.25:shift_end;
y=linspace(a,a,n);    
plot3(k,y,wj(2:3,1:n));
end


