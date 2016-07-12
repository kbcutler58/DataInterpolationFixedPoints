%% Load in Mesh and Calibration Points
clear
clc
cd('C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input');
% [Vertices,Faces] = read_vertices_and_faces_from_obj_file('meshDefinitino_ForearmGrid1.obj');
[Vertices,Faces] = read_vertices_and_faces_from_obj_file('meshDefinition_KyleForearm.obj');
% calibPoints = importdata('fixedPoints_ForearmGrid1.txt');
calibPoints = importdata('fixedPoints_KyleForearm.txt');
%Average Calibration Vertices
for i = 1:length(calibPoints)/4
    range = (1+4*(i-1):4*i);
    calibPointsFinal(i,1) = mean(calibPoints(range,1));
    calibPointsFinal(i,2) = mean(calibPoints(range,2));
    calibPointsFinal(i,3) = mean(calibPoints(range,3));
end
%% Display Mesh and Calibration Points
% close all
% hold on
% scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3),30,'filled','r')
meshScale = 80;
Vertices2 = Vertices*meshScale;
oldCenter(1) = .5*(max(Vertices2(:,1))+min(Vertices2(:,1)));
oldCenter(2) = .5*(max(Vertices2(:,2))+min(Vertices2(:,2)));
oldCenter(3) = .5*(max(Vertices2(:,3))+min(Vertices2(:,3)));
Vertices3(:,1) = Vertices2(:,1) - oldCenter(1);
Vertices3(:,2) = Vertices2(:,2) - oldCenter(2);
Vertices3(:,3) = Vertices2(:,3) - oldCenter(3);
% hold on
% trisurf(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
% trimesh(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
%% Select Vertices from Data
% 
% time1 = '2016_06_10__14_11_31';
% time2 = '2016_06_10__14_13_03';
% time3 = '2016_06_10__14_14_32';
% time4 = '2016_06_10__14_16_31';
% time5 = '2016_06_10__14_18_05';
% time6 = '2016_06_10__14_19_52';
% time7 = '2016_06_10__14_21_22';
% posttime1 = '2016_06_10__14_29_47';
% posttime2 = '2016_06_10__14_31_19';
% posttime3 = '2016_06_10__14_32_27';
% posttime4 = '2016_06_10__14_33_08';
% posttime5 = '2016_06_10__14_33_53';
% posttime6 = '2016_06_10__14_34_24';
% posttime7 = '2016_06_10__14_34_48';
% 
% 
% pretime = {time1, time2, time3, time4, time5, time6, time7};
% posttime = {posttime1, posttime2, posttime3, posttime4, posttime5, posttime6, posttime7};

% % %1cm Spacing Grid
% time1 = '2016_06_14__17_52_33';
% posttime1 = '2016_06_14__19_54_51';
% time2 = '2016_06_14__20_27_54';
% posttime2 = '2016_06_14__20_30_05';
% time3 = '2016_06_14__20_36_19';
% posttime3 = '2016_06_14__20_38_17';
% time4 = '2016_06_14__20_41_52';
% posttime4 = '2016_06_14__20_42_52';
% 
% pretime ={time4,time3,time2,time1};
% posttime= {posttime4,posttime3,posttime2,posttime1};

% 5mm Spacing Grid

% time1 = '2016_06_15__16_33_12';
% posttime1 = '2016_06_15__16_33_51';
% time2 = '2016_06_15__15_22_27';
% posttime2 = '2016_06_15__15_23_30';
% time3 = '2016_06_15__15_48_44';
% posttime3 = '2016_06_15__15_49_30';
% time4 = '2016_06_15__15_50_43';
% posttime4 = '2016_06_15__15_51_47';
% time5 = '2016_06_15__15_55_31';
% posttime5 = '2016_06_16__13_38_16';
% time6 = '2016_06_15__16_43_15';
% posttime6 = '2016_06_16__13_46_19';
% time7 = '2016_06_15__16_15_57';
% posttime7 = '2016_06_15__16_16_42';
% 
% pretime ={time1,time2,time3,time4,time5,time6,time7};
% posttime= {posttime1,posttime2,posttime3,posttime4,posttime5,posttime6,posttime7};


time1 = '2016_06_16__14_43_42';
posttime1 = '2016_06_16__14_45_29';

pretime = {time1};
posttime = {posttime1};
% time = '2016_06_14__20_27_54';
% posttime2 = '2016_06_14__20_30_05';
% time3 = '2016_06_14__20_36_19';
% posttime3 = '2016_06_14__20_38_17';
% time4 = '2016_06_14__20_41_52';
% posttime4 = '2016_06_14__20_42_52';
% 
% pretime = {time4,time3,time2,time1};
% posttime= {posttime4,posttime3,posttime2,posttime1};


% time1 = '2016_06_10__15_39_37';
% time2 = '2016_06_10__15_40_34';
% time3 = '2016_06_10__15_41_31';
% time4 = '2016_06_10__15_42_33';
% 
% posttime1 = '2016_06_10__15_46_45';
% posttime2 = '2016_06_10__15_47_11';
% posttime3 = '2016_06_10__15_47_42';
% posttime4 = '2016_06_10__15_48_02';
% 
% 
% pretime = {time1, time2, time3, time4};
% posttime = {posttime1, posttime2, posttime3, posttime4};



%{
% %% Load in Vertex Locations from Tracking Output
% % pretimeStamp1 = '2016_06_09__15_10_52';
% % pretimeStamp2 = '2016_06_09__15_11_43';
% % posttimeStamp1 = '2016_06_09__15_17_47';
% % posttimeStamp2 = '2016_06_09__15_18_36';
% pretimeStamp1 = time1;
% pretimeStamp2 = time2;
% pretimeStamp3 = time3;
% pretimeStamp4 = time4;
% pretimeStamp5 = time5;
% pretimeStamp6 = time6;
% pretimeStamp7 = time7;
% posttimeStamp1 = posttime1; 
% posttimeStamp2 = posttime2;
% posttimeStamp3 = posttime3; 
% posttimeStamp4 = posttime4;
% posttimeStamp5 = posttime5; 
% posttimeStamp6 = posttime6;
% posttimeStamp7 = posttime7; 
% 
% 
% postRotatedPath = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_RecordedPathRendering\output';
% % 2016_06_02__17_53_25
% preRotatedPath = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\output';
% % 2016_06_02__17_47_02
% % % preRotatedPathTimeStamp = '2016_06_01__15_17_08';
% % preRotatedPathTimeStamp = '2016_06_02__17_25_37';
% % % preRotatedPathTimeStamp = '2016_06_02__17_47_02';
% % preRotatedPathTimeStamp2 = '2016_06_02__17_25_37';
% % % postRotatedPathTimeStamp = '2016_06_01__16_02_53';
% % % postRotatedPathTimeStamp ='2016_06_02__17_27_14';
% % postRotatedPathTimeStamp ='2016_06_02__17_53_25';
% % % postRotatedPathTimeStamp ='2016_06_07__22_37_38';
% % postRotatedPathTimeStamp2 ='2016_06_02__17_53_25';
% % 
% 
% preRotatedPathTimeStamp = pretimeStamp1; 
% preRotatedPathTimeStamp2 = pretimeStamp2;
% postRotatedPathTimeStamp = posttimeStamp1;
% postRotatedPathTimeStamp2 = posttimeStamp2;
% cd(postRotatedPath);
% % file1 = strcat('verticesAndIndexOutput_',char(postRotatedPathTimeStamp),'.txt');
% file1 = strcat('currentPathVertices_',char(postRotatedPathTimeStamp),'.txt');
% set1 = importdata(file1);
% set1(:,5) = 0;
% set1(1:2:end,5) = 1:1:round(length(set1)/2)
% cd(preRotatedPath);
% file2 = strcat('xyzVerticesAndSignalData_',char(preRotatedPathTimeStamp),'.txt');
% set2 = importdata(file2);
% buttonPress = find(set2(:,4) == 0);
% buttonPressDebounce =[];
% for i = 1:length(buttonPress)
%     if i == 1
%         buttonPressDebounce(1) = buttonPress(i);
%         temp = buttonPress(i);
%     else        
%         if buttonPress(i) - temp > 100
%         buttonPressDebounce = [buttonPressDebounce buttonPress(i)];
%         temp = buttonPress(i);
%         end
%     end
% end
% file3 = strcat('xyzVerticesTimestampSignalInfo_',char(preRotatedPathTimeStamp),'.txt');
% set3 = importdata(file3);
% timeStampOriginal = set3(1,5);
% timeStamps= set3(buttonPressDebounce,5);
% file4 = strcat('compressedVerticesAndSignalInfo_',char(preRotatedPathTimeStamp),'.txt');
% set4 = importdata(file4);
% 
% for i = 1:length(timeStamps)
%    [~,value] = min(abs(set4(:,5)-timeStamps(i)));
%     compressedIndex(i) = value;
%     PostcompressedIndex(i) = find(set1(:,5)==compressedIndex(i));
%     PostcompressedVert(i,1:3) = set1(PostcompressedIndex(i),1:3);
% end
%}

postRotatedPath = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_RecordedPathRendering\output';
% % 2016_06_02__17_53_25
preRotatedPath = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\output';
PostcompressedVert = [];
for j = 1:length(pretime)
    postRotatedPathTimeStamp = posttime{j};
    preRotatedPathTimeStamp = pretime{j};
cd(postRotatedPath);
file1 = strcat('currentPathVertices_',char(postRotatedPathTimeStamp),'.txt');
set1 = importdata(file1);
set1(:,5) = 0;
set1(1:2:end,5) = 1:1:round(length(set1)/2)
cd(preRotatedPath);
file2 = strcat('xyzVerticesAndSignalData_',char(preRotatedPathTimeStamp),'.txt');
set2 = importdata(file2);
if size(set2,2)<2
    formatSpec = '%f%f%f%f%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%*s%[^\n\r]';
    fileID = fopen(file2,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', ',')
    fclose(fileID);
    set2 = [dataArray{1,1},dataArray{1,2},dataArray{1,3},dataArray{1,4}];
end
buttonPress = find(set2(:,4) == 0);
buttonPressDebounce =[];
for i = 1:length(buttonPress)
    if i == 1
        buttonPressDebounce(1) = buttonPress(i);
        temp = buttonPress(i);
    else        
        if buttonPress(i) - temp > 40
        buttonPressDebounce = [buttonPressDebounce buttonPress(i)];
        temp = buttonPress(i);
        end
    end
end
file3 = strcat('xyzVerticesTimestampSignalInfo_',char(preRotatedPathTimeStamp),'.txt');
set3 = importdata(file3);
timeStampOriginal = set3(1,5);
timeStamps= set3(buttonPressDebounce,5);
file4 = strcat('compressedVerticesAndSignalInfo_',char(preRotatedPathTimeStamp),'.txt');
set4 = importdata(file4);
vertLength = length(PostcompressedVert);
for i = 1:length(timeStamps)
   [~,value] = min(abs(set4(:,5)-timeStamps(i)));
    compressedIndex(i) = value;
    PostcompressedIndex(i) = find(set1(:,5)==compressedIndex(i));
    PostcompressedVert(i+vertLength,1:3) = set1(PostcompressedIndex(i),1:3);
end
end

scatter3(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3),100,'filled','MarkerEdgeColor','k')
hold on
h = trisurf(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
set(h,'FaceColor',[1 1 1])
view(22.5,-80)
zoom(1.5)
axis off
%% Load in Optical Data
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\trial2_160520\trial2_160520';
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\test_160520\test_160520';
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\grid1_160602\grid1_160602';
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\grid1_160609\grid1_160609';
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\grid1_160609 (2)\grid1_160609';

% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\densegrid1_160610\densegrid1_160610';
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\sparsegrid2_160610\sparsegrid2_160610'
% opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\grid4_160614\grid4_160614';
opticalDataPath = 'C:\Users\BLI\Desktop\Downloads\grid5_160614\grid5_160614';

cd(opticalDataPath)
% fid2=fopen('pttrial2_160520_trial2_pathrecorded__TIME.asc');
% fid2=fopen('pttest_160520_highresGrid1__TIME.asc');

% fid2=fopen('ptdensegrid1_160610_denseGrid1__TIME.asc');
% fid2=fopen('ptsparsegrid2_160610_sparseGrid1__TIME.asc');
% fid2=fopen('ptgrid4_160614_normGrid1__TIME.asc');
fid2=fopen('ptgrid5_160614_denseGrid1__TIME.asc');

opticalTimes=textscan(fid2,'%s %s');
fclose('all');
[a,b]=sort(opticalTimes{2}(2:end));
% fid = fopen('pttrial2_160520_trial2_pathrecorded__SUM.asc');
% fid = fopen('pttest_160520_highresGrid1__SUM.asc');

% fid = fopen('ptdensegrid1_160610_denseGrid1__SUM.asc');
% fid = fopen('ptsparsegrid2_160610_sparseGrid1__SUM.asc');
% fid = fopen('ptgrid4_160614_normGrid1__SUM.asc');
fid = fopen('ptgrid5_160614_denseGrid1__SUM.asc');

opticalStringData = textscan(fid, '%s');
fclose('all');
for i=1:size(opticalStringData{1,1})
      ind1(i)=strcmp(opticalStringData{1,1}(i),'wavelength(nm)');
      ind2(i)=strcmp(opticalStringData{1,1}(i),'FDPM');
      ind3(i)=strcmp(opticalStringData{1,1}(i),'HbO2');
end
startIndex = find(ind1==1)+1;
endIndex = find(ind2==1,1);
numPoints = endIndex-startIndex;
chromIndex = find(ind3==1);
count=1;
for j = 1:numPoints
for i = b(j)
    chromMat(count,1) = str2num(char(opticalStringData{1,1}(chromIndex+i)));
    chromMat(count,2) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints+i+1)));
    chromMat(count,3) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*2+i+2)));
    chromMat(count,4) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*3+i+3)));
    chromMat(count,5) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*4+i+4)));
    chromMat(count,6) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*5+i+5)));
    chromMat(count,7) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*6+i+6)));
    count=count+1;
end
end

%% Select Region for Interpolation

xmin = min(calibPointsFinal(:,1));
xmax = max(calibPointsFinal(:,1));
ymin = min(calibPointsFinal(:,2));
ymax = max(calibPointsFinal(:,2));
zmin = min(calibPointsFinal(:,3));
zmax = max(calibPointsFinal(:,3));

xmin = min(PostcompressedVert(:,1));
xmax = max(PostcompressedVert(:,1));
ymin = min(PostcompressedVert(:,2));
ymax = max(PostcompressedVert(:,2));
zmin = min(PostcompressedVert(:,3));
zmax = max(PostcompressedVert(:,3));

factor = 0;
condition1 = Vertices3(:,1) > xmin-factor;
condition2 = Vertices3(:,1) < xmax+factor;
condition3 = Vertices3(:,2) > ymin-factor;
condition4 = Vertices3(:,2) < ymax+factor;
condition5 = Vertices3(:,3) > zmin-factor;
condition6 = Vertices3(:,3) < zmax+factor;
interpRange = condition1&condition2&condition3&condition4&condition5&condition6;

interpRange2 = zeros(length(Vertices3),1);
for i = 1:length(PostcompressedVert)
    distance(:,1) = (Vertices3(:,1)-PostcompressedVert(i,1)).^2;
    distance(:,2) = (Vertices3(:,2)-PostcompressedVert(i,2)).^2;
    distance(:,3) = (Vertices3(:,3)-PostcompressedVert(i,3)).^2;
    distance = sqrt(distance(:,1)+distance(:,2)+distance(:,3));          
    [temp1,temp2] = find(distance < .4);
    interpRange2(temp1) = 1  ;  
end
interpRange2 = logical(interpRange2);
scatter3(Vertices3(interpRange2,1),Vertices3(interpRange2,2),Vertices3(interpRange2,3))
interpRange = interpRange2;

% % Plot interpolation region
hold on
scatter3(Vertices3(interpRange,1),Vertices3(interpRange,2),Vertices3(interpRange,3))
trimesh(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3))
scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3),100,'filled','r')

[part1,part2]=find(interpRange==1);
interpFaceRange = zeros(length(Faces),1);
for i = 1:length(Faces)
    if Faces(i,:) <= max(part1)
        if Faces(i,:) >= min(part1)
            interpFaceRange(i) = 1;
        end
    end
end
% Faces(logical(interpFaceRange),:)
% trisurf(Faces(logical(interpFaceRange),:),Vertices3(interpRange,1),Vertices3(interpRange,2),Vertices3(interpRange,3))

%% Interpolate Data
% condition1&condition2 &&condition3 && condition4 &&condition5 && condition6
chromophoreSelect = 2; % 1 Oxyhemo 2 Deoxyhemo 3 Fat 4 Lipid
V = chromMat(1:length(PostcompressedVert),chromophoreSelect);

% VF = scatteredInterpolant(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3),V,'linear');
% VFx = VF(Vertices3(interpRange,1),Vertices3(interpRange,2),Vertices3(interpRange,3));

    % VFx = VF(Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
    % VFx = VF(Vertices3(:,1),Vertices3(:,2));
    % VF = VF2(Vertices3(interpRange,1),Vertices3(interpRange,2));

    VF = scatteredInterpolant(PostcompressedVert(:,1),PostcompressedVert(:,2),V,'linear');
    VFx = VF(Vertices3(interpRange,1),Vertices3(interpRange,2));
%     VFx = VF(Vertices3(:,1),Vertices3(:,2));
% close all
% figure(1)
figure('units','normalized','outerposition',[0 0 .5 1])

% White Mesh Option
hold on
h = trisurf(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
set(h,'FaceColor',[1 1 1])

% Textured Mesh Option
cd('C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input');
filename = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input\Faces1.txt';
delimiter = {' ','/'};
formatSpec = '%*s%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
Faces1 = [dataArray{1:end-1}];
clearvars filename delimiter formatSpec fileID dataArray ans;
textCoordinates = importdata('textureCoordinates1.txt');
texture1 = imread('textureDefinition_KyleForearm.png');
Options.PSize = 128;
endRange = 15300;
patcht(Faces(1:endRange,:),Vertices3,Faces1(1:endRange,[2 5 8]),textCoordinates.data,texture1,Options)
hold on

% trimesh(Faces,Vertices3(interpRange,1),Vertices3(interpRange,2),Vertices3(interpRange,3),VFx)
% % Plot Path Lines
% plot3(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3),'b','LineWidth',3)

view(22.5,-80)
zoom(2.2)
axis off

[projectionFactorX,projectionFactorY,projectionFactorZ] = sph2cart(22.5,-80,-.2);
% Plot all distinct data points
scatter3(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3)-.15,100,'filled','MarkerEdgeColor','k')
% scatter3(PostcompressedVert(:,1)+projectionFactorX,PostcompressedVert(:,2)+projectionFactorY,PostcompressedVert(:,3)+projectionFactorZ,100,V,'MarkerEdgeColor','k','LineWidth',2)

%  scatter3(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3),100,'kv','filled','MarkerEdgeColor','k')

% Scatter Plot Calibration Points
scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3)-.2,150,'filled','k')

% Scatter Plot Interpolated Data
scatter3(Vertices3(interpRange,1),Vertices3(interpRange,2),Vertices3(interpRange,3),400,VFx,'s','filled');

% Adjustment of Colormap/bar
colormap('jet')
% caxis([0, 250])
% caxis([0, 100])
caxis([0, 22])
% caxis([0, max(V)*1.2])
colorbar

% Adjustment of view
% view(22.5,-80)
% zoom(1.5)
% axis off

% title('02Hb Distribution Forearm')

% patchTexture(h,texture1)
% %%
% close all
% scatter3(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3),100,V,'s','filled','MarkerEdgeColor','k')

%% Show downsampled version
clear VF VFx V

% close all
figure('units','normalized','outerposition',[0 0 .5 1])

hold on
h = trisurf(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
set(h,'FaceColor',[1 1 1])

% % Textured Mesh Option
% cd('C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input');
% filename = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input\Faces1.txt';
% delimiter = {' ','/'};
% formatSpec = '%*s%f%f%f%f%f%f%f%f%f%[^\n\r]';
% fileID = fopen(filename,'r');
% dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN, 'ReturnOnError', false);
% fclose(fileID);
% Faces1 = [dataArray{1:end-1}];
% clearvars filename delimiter formatSpec fileID dataArray ans;
% textCoordinates = importdata('textureCoordinates1.txt');
% texture1 = imread('textureDefinition_KyleForearm.png');
% Options.PSize = 128;
% endRange = 15300;
% patcht(Faces(1:endRange,:),Vertices3,Faces1(1:endRange,[2 5 8]),textCoordinates.data,texture1,Options)
% hold on

downSampledRange = [1:2:9 19:2:27 37:2:45 55:2:63];
chromophoreSelect = 2; % 1 Oxyhemo 2 Deoxyhemo 3 Fat 4 Lipid
V = chromMat(downSampledRange,chromophoreSelect);
VF = scatteredInterpolant(PostcompressedVert(downSampledRange,1),PostcompressedVert(downSampledRange,2),V,'linear','none');
VFx = VF(Vertices3(interpRange,1),Vertices3(interpRange,2));
hold on
scatter3(Vertices3(interpRange,1),Vertices3(interpRange,2),Vertices3(interpRange,3),400,VFx,'s','filled');
scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3)-.2,150,'filled','k')
scatter3(PostcompressedVert(downSampledRange,1)+projectionFactorX,PostcompressedVert(downSampledRange,2)+projectionFactorY,PostcompressedVert(downSampledRange,3)+projectionFactorZ,100,V,'MarkerEdgeColor','k','LineWidth',2);
view(22.5,-80)
zoom(2.2)
% zoom(1.2)
axis off
colormap('jet')
% caxis([0, 250])
% caxis([0, 100])
caxis([0, 22])

% caxis([0, max(V)*1.1])
colorbar

%%
time1 = '2016_06_16__14_43_42';
posttime1 = '2016_06_16__14_45_29';
calibPoints = importdata('fixedPoints_KyleForearm - Copy.txt');
%Average Calibration Vertices
for i = 1:length(calibPoints)/4
    range = (1+4*(i-1):4*i);
    calibPointsFinal(i,1) = mean(calibPoints(range,1));
    calibPointsFinal(i,2) = mean(calibPoints(range,2));
    calibPointsFinal(i,3) = mean(calibPoints(range,3));
end
figure('units','normalized','outerposition',[0 0 .5 1])

% Textured Mesh Option
cd('C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input');
filename = 'C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input\Faces1.txt';
delimiter = {' ','/'};
formatSpec = '%*s%f%f%f%f%f%f%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
Faces1 = [dataArray{1:end-1}];
clearvars filename delimiter formatSpec fileID dataArray ans;
textCoordinates = importdata('textureCoordinates1.txt');
texture1 = imread('textureDefinition_KyleForearm.png');
Options.PSize = 128;
endRange = 15300;
patcht(Faces(1:endRange,:),Vertices3,Faces1(1:endRange,[2 5 8]),textCoordinates.data,texture1,Options)
hold on


% hold on
% h = trisurf(Faces,Vertices3(:,1),Vertices3(:,2),Vertices3(:,3));
% set(h,'FaceColor',[1 1 1])

scatter3(PostcompressedVert(:,1),PostcompressedVert(:,2),PostcompressedVert(:,3)-.1,100,'r','LineWidth',2)
plot3(set1(:,1),set1(:,2),set1(:,3)-.15,'b','LineWidth',4)
view(22.5,-80)
zoom(2.2)
scatter3(calibPointsFinal(1:4,1),calibPointsFinal(1:4,2),calibPointsFinal(1:4,3)-.2,150,'filled','k')
scatter3(calibPointsFinal(5:9,1),calibPointsFinal(5:9,2),calibPointsFinal(5:9,3)-.1,150,'filled','y')
distanceError(1) = sqrt(sum((PostcompressedVert(1,:) - calibPointsFinal(5,:)).^2))
distanceError(2) = sqrt(sum((PostcompressedVert(2,:) - calibPointsFinal(6,:)).^2))
distanceError(3) = sqrt(sum((PostcompressedVert(3,:) - calibPointsFinal(7,:)).^2))
distanceError(4) = sqrt(sum((PostcompressedVert(4,:) - calibPointsFinal(8,:)).^2))
distanceError(5) = sqrt(sum((PostcompressedVert(5,:) - calibPointsFinal(9,:)).^2))
finalError = distanceError/0.08203125;

distanceSum(1) = sqrt(sum((calibPointsFinal(1,:) - calibPointsFinal(5,:)).^2))
distanceSum(2) = sqrt(sum((calibPointsFinal(5,:) - calibPointsFinal(6,:)).^2))
distanceSum(3) = sqrt(sum((calibPointsFinal(6,:) - calibPointsFinal(7,:)).^2))
distanceSum(4) = sqrt(sum((calibPointsFinal(7,:) - calibPointsFinal(8,:)).^2))
distanceSum(5) = sqrt(sum((calibPointsFinal(8,:) - calibPointsFinal(9,:)).^2))
finalDistances = distanceSum/.08203125;
finalDistancesSum = cumsum(finalDistances)'

distanceEstimatedSum(1) = sqrt(sum((calibPointsFinal(1,:) - PostcompressedVert(1,:)).^2))
distanceEstimatedSum(2) = sqrt(sum((PostcompressedVert(1,:) - PostcompressedVert(2,:)).^2))
distanceEstimatedSum(3) = sqrt(sum((PostcompressedVert(2,:) - PostcompressedVert(3,:)).^2))
distanceEstimatedSum(4) = sqrt(sum((PostcompressedVert(3,:) - PostcompressedVert(4,:)).^2))
distanceEstimatedSum(5) = sqrt(sum((PostcompressedVert(4,:) - PostcompressedVert(5,:)).^2))
finalEstimatedDistances = distanceEstimatedSum/.08203125;
finalEstimatedDistancesSum = cumsum(finalEstimatedDistances)'