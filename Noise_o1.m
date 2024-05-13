function Noise = Noise_o1(k,scenario)
    Noise=demando1(k,scenario)+500*sin(0.03*(k-60)).*((k-60)>=0);
end

