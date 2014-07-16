function [] = Untitled2( h, tf, x0, v0 )

t=0:0.01:tf;

for ii=1:1:length(t)
    y(ii)=h-9.8*(t(ii)^2)
    x(ii)=x0+v0*t(ii)
    plot(x(ii), y(ii), '*b')
    hold on
    pause(0.1)
end


end

