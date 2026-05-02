 clc; 
clear;
%clf(1);
%% 
%%------------------------------------------------------------------
%Robust wireless power transfer using a nonlinear parity–time-symmetric circuit
%------------------------------------------------------------------
%定义方程组如下，其中k为变量
 syms w g k
 w1=1;w2=1 ;r2=0.1;%定义发射端W1与接收端W2，偏离参数在这改
  eqns = (w-w1)*(w-w2)^2 + (w-w1)*r2^2-(w-w2).*k.^2 ; %方程3 
  wr=solve(eqns,w);  
 i=0;n=20; %仿真数据一定要多
 k_start=1e-4; k_end=0.2;%定义k的范围
 for k1=linspace(k_start, k_end, n)
 i=i+1;
 wj(1:3,i)=real(subs(wr, k, k1));
 end
 %wj=sort(wj);
 k=linspace(k_start, k_end, n); 

 %------------------------------------------------------------------
%画标准PT下的四张图，通过修改W1与W2来实现效果
%------------------------------------------------------------------
  figure(1);hold on;
  %plot(k,wj(1,1:n));%  PT对称中间线
  plot(k,wj(2,1:n)); 
  plot(k,wj(3,1:n));
