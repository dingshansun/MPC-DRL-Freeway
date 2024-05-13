function vsl_control = Para_VSL_control_law(theta, vsl_pre, rou, V, rou_pre, V_pre, v_free)
    vsl_control=v_free+theta(1)*(V_pre-V)+theta(2)*(rou_pre-rou);
end

