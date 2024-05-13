function sat_A = sat_vsl(A)
    if A>=102
        sat_A=102;
    elseif A<=20
        sat_A=20;
    else
        sat_A=A;
    end
end

