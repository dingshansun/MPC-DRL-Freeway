function V_ = Speed(V,V_desire,V_up,rou,rou_down,para,q_o,v_min)
%SPEED para=[T,tao,yita,kai,Lm,sigma,lambda];
%   
T=para(1)/3600;
tao=para(2)/3600;
yita=para(3);
kai=para(4);
Lm=para(5)/1000;
sigma=para(6);
lambda=para(7);
V_=max(v_min, V...
    +T/tao*(V_desire-V)...
    +T/Lm*V*(V_up-V)...
    -(yita*T)/(tao*Lm)*(rou_down-rou)/(rou+kai)...
    -sigma*T*q_o*V/(Lm*lambda*(rou+kai)));
end

