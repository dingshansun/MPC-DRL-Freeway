function Noise = Noise_o2(k,scenario)
    Noise=demando2(k,scenario)+200*sin(0.02*(k-60)).*((k-60)>=0);
end

