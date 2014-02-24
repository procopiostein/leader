for i = 1:length(a)
    winsize = 30;
    while(i-winsize <= 0) %not enough past info
        winsize = winsize - 1;
    end
    
    % stdv part
%     calc_mean(i) = std(input_features(i-winsize:i,3));
    
    % mean part
    calc_mean(i) = mean(a(i-winsize:i));
    calc_mean = calc_mean';
end