function sat_A = sat_vslrm(A)
B=min(A,[102 102 102 102 102 102 1 1 1]');
sat_A=max(B,[20 20 20 20 20 20 0 0 0]');
end

