u=[];
for i = 0:7000
    if i < 2000
        u(i+1) = (44./(1+exp(-61*(i/10000)+6)));
        u(9000-i) = u(i+1);
    else
        u(i) = 44;
    end
            
end
x = (0:1:8999);
plot(x/10000,u)