function a = limitRange(num, max, min)
    if num >= max
        a = max;
    else
        if num <= min
            a = min;
        else
            a = num;
        end
    end
end