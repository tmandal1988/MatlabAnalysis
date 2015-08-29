function engr_out = bytes2engr(bytes,precision)
%% Types of Fields
if strcmp(precision,'GPSec')
    precision = 'long';
elseif strcmp(precision,'enum')
    precision = 'ulong';
end
if strcmp(precision,'char')
    engr_out = int8(bytes);
elseif strcmp(precision,'uchar')
    engr_out = char(bytes);
elseif strcmp(precision,'short')
%     engr_out = int16(bitshift(uint16(bytes(2)),8) + uint16(bytes(1))-2^15);
    engr_out = bitshift(uint16(bytes(2)),8)...
        + uint16(bytes(1));
    if bytes(2) > 127   % = negative in two's complement
        engr_out = double(engr_out) - 2^16;
    else
        engr_out = double(engr_out);
    end    
elseif strcmp(precision,'ushort')
    engr_out = double((uint16(bytes(2)))*256 + uint16(bytes(1)));
elseif strcmp(precision,'ushort_nb')
    engr_out = double((uint16(bytes(1)))*256 + uint16(bytes(2)));
elseif strcmp(precision,'long')
    engr_out = bitshift(uint32(bytes(4)),24)...
        + bitshift(uint32(bytes(3)),16)...
        + bitshift(uint32(bytes(2)),8)...
        + uint32(bytes(1));
    if bytes(4) > 127   % = negative in two's complement
        engr_out = double(engr_out) - 2^32;
    else
        engr_out = double(engr_out);
    end
%     engr_out = int32(bitshift(uint32(bytes(4)),24)...
%         + bitshift(uint32(bytes(3)),16)...
%         + bitshift(uint32(bytes(2)),8)...
%         + uint32(bytes(1))-2^31);
elseif strcmp(precision,'ulong')
    engr_out = bitshift(uint32(bytes(4)),24)...
        + bitshift(uint32(bytes(3)),16)...
        + bitshift(uint32(bytes(2)),8)...
        + uint32(bytes(1));
elseif strcmp(precision,'double')
%     binary = '';
%     for i = 8:-1:1
%         byte_bin = dec2bin(bytes(i));
%         while length(byte_bin) < 8
%             byte_bin = ['0',byte_bin];
%         end
%         binary = [binary,byte_bin];
%     end
%     engr_out = ieee7542dec(binary,'double');
   
    double_bin_array = zeros(1, 64);
    for i = 1:8
        byte_bin_array = int2bin_array(double(bytes(9-i)));          
        double_bin_array((i-1)*8+(1:8)) = byte_bin_array;
    end
    engr_out = bin_arrayIEEE7542dec(double_bin_array);
elseif strcmp(precision,'short_nb')
    engr_out = double(typecast(uint16(bytes(1))*256+uint16(bytes(2)),'int16'));%;int16(bitshift(uint16(bytes(2)),8) + uint16(bytes(1))-2^15);
elseif strcmp(precision,'float')
%     binary = '';
%     for i = 4:-1:1
%         byte_bin = dec2bin(bytes(i));
%         while length(byte_bin) < 8
%             byte_bin = ['0',byte_bin];
%         end
%         binary = [binary,byte_bin];
%     end
%     engr_out = ieee7542dec(binary,'single');
    
    float_bin_array = zeros(1, 32);
    for i = 1:4
        byte_bin_array = int2bin_array(double(bytes(5-i)));
        float_bin_array((i-1)*8+(1:8)) = byte_bin_array;
    end
    engr_out = bin_arrayIEEE7542dec(float_bin_array);
elseif strcmp(precision,'hex')
    engr_out = inf;
    %% Not needed at the moment
    %     fprintf('\nWarning: bytes2engr (precision == ''hex'' not written)\n')
elseif strcmp(precision,'string')
    engr_out = inf;
    %% Not needed at the moment
    %     fprintf('\nWarning: bytes2engr (precision == ''string'' not written)\n')
    
end

function bin_array = int2bin_array(x_int)
% Function to convert an 8 bit integer (char) to a 1x8 array of binary
bin_array = zeros(1,8);
for i = 1:8
    bin_array(9-i) = mod(x_int, 2);
    x_int = floor(x_int/2);
end
return

function y = bin_arrayIEEE7542dec(x)
% Function to convert a 1x8 array of binary (IEEE754) to decimal
if length(x) == 32
    eV = x(2:9);
    mV = x(10:32);
    b = 127;
elseif length(x) == 64
    eV = x(2:12);
    mV = x(13:64);
    b = 1023;
else
    disp('Error:  must be single or double precision');
    y = 0;
    return;
end

% Determine sign from first bit
s = 1;
if x(1) == 1
    s = -1;
end

% Check for special cases
if sum(x == 1) == 0
    y = 0;
    return;
end
if sum(eV == 0) == 0
    if s == 0
        y = inf;
        return;
    else
        y = -inf;
        return;
    end
end

e = 0;
ne = length(eV);
for i = 1:ne
    e = e + eV(i)*(2^(ne-i));
end

m = 1;
nm = length(mV);
for i = 1:nm
    m = m + mV(i)*(2^(-i));
end
y = s*m*2^(e-b);

return