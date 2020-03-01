%% Init console
% Go to script folder
filename = which(mfilename);
[pathstr,~,ext] = fileparts(filename);
cd(pathstr);

% Clear console and variables
clc;
clear all;
close all;

%% Parameters
dataFolder = "data_car";

%% Load file
files = dir(fullfile(dataFolder, '*.txt'));

for idx = 1:numel(files)
    file = files(idx);
    f_path = [file.folder, filesep,file.name];
    display(file.name)
    tmp_struct.name = file.name;
    fid = fopen(f_path,'r');
    
    lane_num = -2;
    while ~feof(fid)
        lane_num = lane_num + 1;
        line = fgetl(fid);
        if lane_num < 1
            continue;
        end
        splitted = split(line,"|");
        
        info = split(splitted(1),",");
        tmp_struct.total_kp(lane_num) = str2num(info{1});
        tmp_struct.t_kp(lane_num) = str2num(info{2});
        tmp_struct.car_kp(lane_num) = str2num(info{3});
        tmp_struct.t_desc(lane_num) = str2num(info{4});
        tmp_struct.n_match(lane_num) = str2num(info{5});
        tmp_struct.t_match(lane_num) = str2num(info{6});
        tmp_struct.a_tot(lane_num) = str2num(info{7});
        tmp_struct.a_u(lane_num) = str2num(info{8});
        
        kpts_info = split(splitted(2),";");
        
        kpts = zeros(0,3);
        for i=1:numel(kpts_info)
            kp_info =split(kpts_info{i},",");
            if(numel(kp_info)~=3)
                continue;
            end
            x = str2num(kp_info{1});
            y = str2num(kp_info{2});
            d = str2num(kp_info{3});
            kpts(end+1,:) = [x,y,d];
        end
        
        tmp_struct.kpts{lane_num} = kpts;
    end
    fclose(fid);
    
    if(idx == 1)
        data = tmp_struct;
    else
        data(idx) = tmp_struct;
    end    
end

save("data","data");