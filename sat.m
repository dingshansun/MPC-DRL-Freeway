function sat_A = sat(A)
B=min(A,[102 102 1]');
sat_A=max(B,[20 20 0]');
end

