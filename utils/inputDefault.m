function val = inputDefault(promptStr, defaultVal)

str = input(sprintf('%s [Default=%.4g]: ', promptStr, defaultVal), 's');
if isempty(str)
    val = defaultVal;
else
    val = str2double(str);
    if isnan(val)
        warning('Use default value %.4g。', defaultVal);
        val = defaultVal;
    end
end
end
