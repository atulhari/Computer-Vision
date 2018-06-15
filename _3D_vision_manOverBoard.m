%% Man Overboard
clc;
clear variables;
%Initialize file
filename = 'Project.mp4';
%Reading the Video File
hVideoSource = vision.VideoFileReader(filename, ...
'ImageColorSpace', 'Intensity',...
'VideoOutputDataType', 'double');
%Initialize Template Matching
hTM = vision.TemplateMatcher('ROIInputPort', true, ...
'BestMatchNeighborhoodOutputPort', true);
%Writing the Video File
y = vision.VideoFileWriter('myFile.avi','FrameRate',hVideoSource.info.VideoFrameRate);
%Opening the Video Player
hVideoOut = vision.VideoPlayer('Name', 'Video Stabilization');
%Setting up of Position of the Video Player
hVideoOut.Position(1) = round(0.4*hVideoOut.Position(1));
hVideoOut.Position(2) = round(1.5*(hVideoOut.Position(2)));
hVideoOut.Position(3:4) = [650 350];
%Initialize the region
pos.template_orig = [700 500]; % [x y] upper left corner
pos.template_size = [25 20]; % [width height]
pos.search_border = [25 20]; % max horizontal and vertical displacement
pos.template_center = floor((pos.template_size-1)/2);
pos.template_center_pos = (pos.template_orig + pos.template_center - 1);
fileInfo = info(hVideoSource);
W = fileInfo.VideoSize(1); % Width in pixels
H = fileInfo.VideoSize(2); % Height in pixels
BorderCols = [1:pos.search_border(1)+4 W-pos.search_border(1)+4:W];
BorderRows = [1:pos.search_border(2)+4 H-pos.search_border(2)+4:H];
sz = fileInfo.VideoSize;
TargetRowInd = ...
pos.template_orig(2)-1:pos.template_orig(2)+pos.template_size(2)-2;
TargetColInd = ...
pos.template_orig(1)-1:pos.template_orig(1)+pos.template_size(1)-2;
SearchRegion = pos.template_orig - pos.search_border - 1;
%Initialize Offset
Offset = [0 0];
Target = zeros(18,22);
firstTime = true;
while ~isDone(hVideoSource)
%Reading Each frame.
frame = step(hVideoSource);
% Find location of Target in the input video frame
if firstTime
I = int32(pos.template_center_pos);
MotionVector = [0 0];
firstTime = false;
else
IPrev = I;
ROI = [SearchRegion, pos.template_size+2*pos.search_border];
I = step(hTM, frame, Target, ROI);
MotionVector = double(I-IPrev);
end
[Offset, SearchRegion] = updatesearch(sz, MotionVector, ...
SearchRegion, Offset, pos);
% Translate video frame to offset the camera motion
Stabilized = imtranslate(frame, Offset, 'linear');
Target = Stabilized(TargetRowInd, TargetColInd);
% Add black border for display
Stabilized(:, BorderCols) = 0;
Stabilized(BorderRows, :) = 0;
TargetRect = [pos.template_orig-Offset, pos.template_size];
SearchRegionRect = [SearchRegion, pos.template_size + 2*pos.search_border];
% Draw rectangles on input to show target and search region
frame = insertShape(frame, 'Rectangle', [TargetRect; SearchRegionRect],...
'Color', 'white');
% Display the offset (displacement) values on the input image
txt = sprintf('(%+05.1f,%+05.1f)', Offset);
frame = insertText(frame(:,:,1),[191 215],txt,'FontSize',16, ...
'TextColor', 'white', 'BoxOpacity', 0);
% Display video
step(y,Stabilized)
end
release(y);
% % % % % % % Object detection and tracking
clc;
close all;
%% Step 1 - Import Video and Initialize Foreground Detector
foregroundDetector = vision.ForegroundDetector('NumGaussians', 5, ...
'NumTrainingFrames', 150);
%Read Video Frame
videoReader = vision.VideoFileReader('myFile.avi');
for i = 1:200
frame = step(videoReader); % read the next video frame
foreground = step(foregroundDetector, frame);
end
%% output of one of the video frames and the foreground mask computed by the detector.
figure;
imshow(frame);
title('Video Frame');
figure;
imshow(foreground);
title('Foreground');
%% Step 2 - Detect probable buoys in an Initial Video Frame
r_x=[500 700 700 500];
r_y=[500 500 600 600];
r_mask=poly2mask(r_x,r_y,1080,1440);
se = strel('sphere', 1);
filteredForeground = imopen(foreground, se);
filteredForeground2=filteredForeground.*r_mask;
filteredForeground2=logical(filteredForeground2);
figure;
imshow(filtered_Foreground2);
title('Clean Foreground');
%%
%find bounding boxes of each connected component by using vision.BlobAnalysis object.
blobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
'AreaOutputPort', false, 'CentroidOutputPort', false, ...
'MinimumBlobArea', 1, 'MaximumBlobArea', 20);
area = step(blobAnalysis,filteredForeground2)
x=area(1);
y=area(1,2);
r=15;
flag=1;
% To highlight the detected blob and draw box around it
X = insertShape(frame, 'Circle', [x y r], 'Color', 'red');
%%
text_str1=['Buoy Detected'];
X= insertText(X, [540 490],text_str,'FontSize',14,'BoxOpacity',0.4,...
'TextColor','blue');
figure;
imshow(X);
title('Probable Detected Buoy');
% Step 3 - Process the Rest of Video Frames
% In the final step, we process the remaining video frames.
videoPlayer = vision.VideoPlayer('Name', 'Cleaned foreground');
videoPlayer.Position(1:2) = [650,400]; % window size: [width, height]
videoPlayer1 = vision.VideoPlayer('Name', 'Probable Detected Buoys');
videoPlayer1.Position(3:4) = [650,400]; % window size: [width, height]
se = strel('sphere', 1); % morphological filter for noise removal
while ~isDone(videoReader)
frame = step(videoReader); % read the next video frame
% Detect the foreground in the current video frame
foreground = step(foregroundDetector, frame);
% Use morphological opening to remove noise in the foreground
filteredForeground = imopen(foreground, se);
filteredForeground=filteredForeground.*r_mask;
filteredForeground1=logical(filteredForeground);
% Detect the connected components with the specified minimum area, and
% compute their bounding boxes
area = step(blobAnalysis, filteredForeground1);
k=size(area,1);
mn=585;
mx=615;
my=525;
my=535;
q=0;
v=0;
p_x=585;
p_y=529;
switch flag
case 1
for i=1:k-1
if(area(i,1)>=mn) && (area(i,1)<=mx) && (area(i,2)>=my) && (area(i,2)<=my)
p_x=area(i,1);
p_y=area(i,2);
r=15;
result = insertShape(frame, 'Circle', [p_x p_y r], 'Color', 'green');
text_str=['Detected'];
% Display the buoy-like areas found in the video frame
result = insertText(result, [540 490],text_str,'FontSize',14,'BoxOpacity',0.4,...
'TextColor','black');
q=q+1;
disp(p_x)
flag=1;
%%
H=double(cameraParams.ImageSize(1)/2);
cameraHeight = 2.5; %m
earthRadius = 6371008000; %m
cameraToHorizon = sqrt(1 - (earthRadius / (earthRadius + cameraHeight))^2);
% Calculate the angle between the camera center and horizon
% Assuming that the horizon is always above the center of the image
visualAngleHorizon = atan(double(centerToHorizon / cameraParams.FocalLength(2)));
cameraAngle = visualAngleHorizon - cameraToHorizon;
% Calculate the angle between the person and the horizon
centerToPerson = (po_x)-H;
Fraction=double(centerToPerson / cameraParams.FocalLength(2));
personAngle = atan(Fraction);
% Project that angle onto a flat plan to estimate the planar distance to the person
alpha = personAngle - cameraAngle;
beta = pi-(pi/2)-alpha;
distance = cameraHeight * sin(beta) / sin(alpha)
RGB = insertText(frame,[10 10],distance);
imshow(RGB)
else
flag=0;
end
i=i+1;
end
case 0
r=15;
result = insertShape(frame, 'Circle', [p_x p_y r], 'Color', 'red');
text_str=['Predicted'];
% Display the buoy-like areas found in the video frame
result = insertText(result, [540 490],text_str,'FontSize',14,'BoxOpacity',0.4,...
'TextColor','white');
v=v+1;
disp(p_x)
flag=1;
end
disp(flag)
result1 = insertShape(filteredForeground, 'Circle', [p_x p_y 15], 'Color', 'red');
text_str=['Buoy'];
% Display the buoy-like areas found in the video frame
result1= insertText(result1, [540 490],text_str,'FontSize',14,'BoxOpacity',0.4,...
'TextColor','Blue');
% Display the number of probable buoy-like areas found in the video frame
step(videoPlayer, result);
step(videoPlayer1, result1);% display the results
end
release(videoReader); % close the video file
%%