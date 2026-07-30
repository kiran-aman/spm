import simscape.multibody.*;

modelName = "spm.slx";

time = out.input_data.time;
position = out.input_data.signals.values;

clear s;
s = serialport("COM8", 115200);
configureTerminator(s, "CR");
pause(2); % esp32 boot up


for i = 2:size(position, 1)
    data = round(position(i, :) * 509.295818); % 16 microstepping
    writeline(s, num2str(data));
    if s.NumBytesAvailable > 0
        inData = readline(s);
        disp([inData, data]);
    end
    pause(0.01);
end
writeline(s, "0 0 0");
while true
    if s.NumBytesAvailable > 0
        disp([readline(s)]);
    end
end
clear s;
%while true
%    line = readline(s);
%    disp(line);
%    pause(0.005);
%end
