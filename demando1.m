function demand = demando1(k,scenario)
%DEMANDO1 Summary of this function goes here
%   Detailed explanation goes here
% k=k-1;
k=floor((k-60)/6);
switch scenario
    case 1
        t1=105;t2=130; d0=3000; d1=3500; d2=1040;
        demand=d0.*(k<0)+d1.*(k>=0 & k<=t1)+(d1-(d1-d2)/(t2-t1)*(k-t1)).*(k>t1 & k<=t2)+d2.*(k>t2);
    case 3
        t1=105;t2=130; d0=3000; d1=3500; d2=1040;
        demand=d0.*(k<0)+d1.*(k>=0 & k<=t1)+(d1-(d1-d2)/(t2-t1)*(k-t1)).*(k>t1 & k<=t2)+d2.*(k>t2);
end

end

