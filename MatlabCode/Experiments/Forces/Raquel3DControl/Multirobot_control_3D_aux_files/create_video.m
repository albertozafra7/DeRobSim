function create_video(folder,vidtitle,image_names,num_images,fps,init,secs)

%This code assumes the frames for the video are in the current folder and
%they are ordered --that is, the file names end with 1, 2, 3,...).

%Construct video object
outputVideo = VideoWriter(fullfile(strcat(folder,'/',vidtitle,'.avi')));
outputVideo.FrameRate = ceil(fps); %frame rate
open(outputVideo)

%Write images to video

if init==1 %we create an initial segment repeating the initial image during secs seconds (same at the end)

    for i=1:secs*fps
        img = imread(strcat('./',folder,'/',image_names,int2str(1),'.png'));
        writeVideo(outputVideo,img)
    end

end

for ii = 1:num_images-1
   img = imread(strcat('./',folder,'/',image_names,int2str(ii),'.png'));
   writeVideo(outputVideo,img)
end

if init==1 %we create an initial segment repeating the initial image during secs seconds (same at the end)

    for i=1:secs*fps
        img = imread(strcat('./',folder,'/',image_names,int2str(num_images-1),'.png'));
        writeVideo(outputVideo,img)
    end
    
end

%Finalize the video file
close(outputVideo)