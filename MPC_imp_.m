function u_mpc = MPC_imp_(x,u_pre,scenario)
    ramp_only=0;
    Np=2; % min
    Nc=2;  % min
    M=30;
%     N=900;
    N_max=30; % multi-start point
%     xx=zeros(size(x,1),N);
    [~, ~, ~, ~, ~, ~, v_free, ~, ~, T, lambda, Lm, ~, ~, ~, xi_r, xi_s] = parameters_esti;
    dim_speed=2;
    dim_ramp=1;
%     U=zeros(dim_speed+dim_ramp,N);

    para=[Lm, lambda, v_free, M, dim_speed, T, xi_r, xi_s];
    options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);
    if (ramp_only)
        dim_control=dim_ramp;
        v_control=repmat(100*ones(dim_speed,1),1,Nc);
        v_control_pre=v_control(:,1);
        u_pre=ones(dim_control,Nc); % 
        u_lb=repmat(zeros(dim_control,1),1,Nc);
        u_ub=repmat(ones(dim_control,1),1,Nc);
    else
        v_control=[];
        v_control_pre=[];
        dim_control=dim_ramp+dim_speed;
        u_lb=repmat([20*ones(dim_speed,1);zeros(dim_ramp,1)],1,Nc);
        u_ub=repmat([v_free*ones(dim_speed,1);ones(dim_ramp,1)],1,Nc);
%         u_pre=[70*ones(dim_speed,Nc);ones(dim_ramp,Nc)]; % 
    end
    objfun=@(co) ModelState(x, [v_control; co], Nc, Np, para, [v_control_pre;u_pre(:,1)],scenario);
    confun=@(co) ModelCons(x, [v_control; co], Nc, Np, M, scenario);
    u0=cell(1,N_max);u_t=cell(1,N_max);Fval=nan(1,N_max);
    parfor l=1:N_max
        if l==1
            u0{l}=u_pre;
        elseif l==2
            u0{l}=u_lb;
        elseif l==3
            u0{l}=u_ub;
        else
            if (ramp_only)
                u0{l}=repmat(rand,dim_control, Nc);
            else
                u0{l}=repmat([rand(dim_speed,1)*80+20;rand(dim_ramp,1)],1, Nc);
%                 u0{l}=[rand(dim_speed,Nc)*80+20;rand(dim_ramp,Nc)];
            end
        end
        [u_t{l}, Fval(l), exitflag] = fmincon(objfun, u0{l}, [], [], [], [], u_lb, u_ub, confun, options);
        if exitflag == 0
%             warning('the solver does not succeed');
            Fval(l)=1e6;
        elseif exitflag < 0
%             warning('the solver does not succeed');
            Fval(l)=1e6;
        end
    end
    [fval, index]=min(Fval);
    u_opt=[v_control;u_t{index}];
    u_mpc=u_opt;
end
function J=ModelState(x, u, Nc, Np, para, u_pre, scenario)
    Lm=para(1);
    Lambda=para(2);
    v_free=para(3);
    M=para(4);
    dim_speed=para(5);
    T=para(6);
    xi_r=para(7);
    xi_s=para(8);
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,M);
    u_diff=[u_pre u];
    yy=nan(size(x,1),Np*M);
    Diff=zeros(size(u,1),Nc); % the erros between the successive control inputs
    for i=1:Nc
        Diff(:,i)=u_diff(:,i+1)-u_diff(:,i);
    end
    for i=1:Np*M
        x=Freeway_model(x,uc_full(:,i), scenario);
        yy(:,i)=x;
    end
    rou_11=yy(1,:);rou_12=yy(4,:);rou_13=yy(7,:);rou_14=yy(10,:);w_o1=yy(14,:);w_o2=yy(16,:);rou_21=yy(17,:);rou_22=yy(20,:);
    TTS=(((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22))*Lm/1000*Lambda+(w_o1+w_o2))*T/3600;
    Diff_speed=Diff(1:dim_speed,:);
    Diff_r=Diff(dim_speed+1:end,:);
    penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
    J=sum(TTS)+penalty;
end

function [c, ceq]=ModelCons(x, u, Nc, Np, M, scenario)
    ceq=[];
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,M);
    yy=zeros(size(x,1),Np*M);
    for i=1:Np*M
        x=Freeway_model(x,uc_full(:,i),scenario);
        yy(:,i)=x;
    end
    w_o1=yy(14,:);w_o2=yy(16,:);
    c=[w_o1'-200;w_o2'-100];
end