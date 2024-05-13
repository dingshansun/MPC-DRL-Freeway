function demand = demando2(k,scenario)
%DEMANDO2 Summary of this function goes here
%   Detailed explanation goes here
% k=k-1;
k=floor((k-60)/6);
switch scenario
    case 1
        t1=5;t2=t1+6;t3=t2+30;t4=t3+6; d1=300;d2=1500;
        demand=500.*(k<0)+d1.*(k>=0 & k<t1)+(d1+(d2-d1)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+d2.*(k>t2 & k<=t3)...
            +(d2-(d2-d1)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+d1.*(k>t4);
    case 3
        t1=5;t2=t1+6;t3=t2+30;t4=t3+6; d1=400;d2=1300;
        demand=500.*(k<0)+d1.*(k>=0 & k<t1)+(d1+(d2-d1)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+d2.*(k>t2 & k<=t3)...
            +(d2-(d2-d1)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+d1.*(k>t4);
end

end

