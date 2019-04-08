function [D]=SD_READER(fname,data_length)

if isa(fname,'char')
    fid = fopen(fname,'r');
    c = fread(fid,'int32');
    D = zeros(floor(length(c)/data_length-1)+1,data_length);
    for i = 1:length(c)/data_length
        D(i,:) = c(i*(data_length)-(data_length-1):i*(data_length));
    end
    fclose(fid);
else
    disp('Invalid name')
end
