clear all
close all
clc
vr=VideoReader('vid2.avi')
Frate=vr.Framerate;
Nfrm_movie = floor(vr.Duration * vr.FrameRate);% number of frames in movie
% Video writing steps-part 1
%make working directory to store annotated images
workingDir1 = 'Particle_new_vid2';
% workingDir2 = 'Mask_new5';
mkdir(workingDir1)
% mkdir(workingDir2)
mkdir(workingDir1,'images')
% mkdir(workingDir2,'images')
mykvid=[];
X_prev=[];
%% Initialise tracks
tracks = struct(...
    'id', {}, ...
    'bbox', {}, ...
    'kalmanFilter', {}, ...
    'age', {}, ...
    'totalVisibleCount', {}, ...
    'consecutiveInvisibleCount', {},...
    'frequency',{},...
    'velo',{},...
    'particles',[]); % Create an empty array of tracks.
nextId = 1; % ID of the next track
NOF=vr.NumberOfFrames;
for kk=1:1074
    F2=read(vr,kk);
    % figure,imshow(F2)
    % f=imcrop(F2,[1 57 1280 541]);
    f=imcrop(F2,[1 57 1280 541]);
    F=rgb2gray(f);
    bf=(F>190);%fixing lightning threshold
    % figure,imshow(bf);
    %% Detections
    cc=bwlabeln(bf,8);
    cc2=bwconncomp(bf);
    s=regionprops(cc2,'Centroid','BoundingBox');
    centroids=cat(1,s.Centroid);
    boundboxes=cat(1,s.BoundingBox);
    %clustering of centroids of connected components
    c=[centroids zeros(length(centroids(:,1)),1)];
    label=0;
    D0=100;%threshold distance to cluster chnged from 200 to 100
    for i=1:size(c,1)
        x0=c(i,1);y0=c(i,2);
        if (c(i,3)==0)
            c(i,3)=label+1;
            label=label+1;
        end
        for j=i+1:size(c,1)
            x=c(j,1);y=c(j,2);
            D= sqrt((x-x0).^2+(y-y0).^2);
            if (D<D0) && (c(j,3)==0)
                c(j,3)=c(i,3);
            elseif (D<D0) && (c(j,3)~=0)
                c(j,3)=min(c(j,3),c(i,3));
            end
        end
    end
    cpixel=ceil(c);
    bpixel=[ceil(boundboxes) cpixel(:,3)];
    %find the bounding box of each region -- it is available in bb
    n=1;nmax=max(bpixel(:,5));bb=[];
    while(n<=nmax)
        h=1;w=1;
        minx=inf;miny=inf;maxx=1;maxy=1;
        for i=1:size(bpixel,1)
            if (bpixel(i,5)==n)
                minx=min(bpixel(i,1),minx);
                miny=min(bpixel(i,2),miny);
                maxx=max(bpixel(i,1),maxx);
                maxy=max(bpixel(i,2),maxy);
            end
        end
        h=maxx+4-minx;
        w=maxy+4-miny;
        bb=[bb;minx miny h w n];
        n=n+1;
    end
    %centroid of centroids
    ccoid=[];
    freq=zeros(max(cpixel(:,3)),1);
    for p=1:max(cpixel(:,3))
        xsum=0;ysum=0;n=0;
        for b=1:size(cpixel(:,1))
            if (cpixel(b,3)==p)
                xsum=xsum+cpixel(b,1);
                ysum=ysum+cpixel(b,2);
                n=n+1;
            end
        end
        ccoid=[ccoid;xsum/n ysum/n p];
        freq(p)=n;
    end
    ccoid=ceil(ccoid(:,1:2));
    bb(:,5)=freq;
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% predictNewLocationsOfTracks();
    for i = 1:length(tracks)
        BB= tracks(i).bbox;
        % Predict the current location of the track.
        predictedCentroid = predict(tracks(i).kalmanFilter);
        % Forecasting
        tracks(i).particles= update_particles(F_update, Xstd_pos, Xstd_vec, tracks(i).particles);
        %
        % Shift the bounding box so that its center is at
        % the predicted location.
        %             predictedCorner = int32(predictedCentroid) - int32(BB(3:4)) / 2;
        %             tracks(i).bbox = [predictedCorner, BB(3:4),BB(5)];
    end
    %% Assign detections to tracks
    nTracks = length(tracks);
    nDetections = size(ccoid, 1);
    % Compute the cost of assigning each detection to each track.
    cost = zeros(nTracks, nDetections);
    %costp = zeros(nTracks, nDetections);
    for i = 1:nTracks
        cost(i, :) = distance(tracks(i).kalmanFilter, ccoid);%covariance btw centroid obj and present detection
        %costp(i,:)=cost(i,:)./ccoid;
    end
    % Solve the assignment problem.
    costOfNonAssignment = 20;
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    %% Update AssignedTracks();
    numAssignedTracks = size(assignments, 1);
    for i = 1:numAssignedTracks
        trackIdx = assignments(i, 1);
        detectionIdx = assignments(i, 2);
        CC = ccoid(detectionIdx, :);
        BB = bb(detectionIdx, :); %changed bboxes to bb
        % Correct the estimate of the object's location
        % using the new detection.
        correct(tracks(trackIdx).kalmanFilter, CC);
        % Calculating Log Likelihood
        L = calc_log_likelihood(Xstd_rgb, Xrgb_trgt, tracks(trackIdx).particles(1:2,:), bf);
        
        % Resampling
        tracks(trackIdx).particles=resample_particles(tracks(trackIdx).particles,L);
        % Replace predicted bounding box with detected
        % bounding box.
        tracks(trackIdx).bbox = BB;% this also contains frequency at that frame
        tracks(trackIdx).frequency=tracks(trackIdx).frequency+BB(:,5); %this contains the total frequency of lightning for each track
        % Update track's age.
        tracks(trackIdx).age = tracks(trackIdx).age + 1;
        % Update visibility.
        tracks(trackIdx).totalVisibleCount = ...
            tracks(trackIdx).totalVisibleCount + 1;
        tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
    %%   Update unassigned tracks
    for i = 1:length(unassignedTracks)
        ind = unassignedTracks(i);
        tracks(ind).age = tracks(ind).age + 1;
        tracks(ind).consecutiveInvisibleCount = ...
            tracks(ind).consecutiveInvisibleCount + 1;
    end
    %%     deleteLostTracks() %if making into a function, make use of return command
    if ~(isempty(tracks))
        invisibleForTooLong = 15;
        ageThreshold = 8;
        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;
        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.8) | ...%so basically it means 0.6*8=4.8, visibility less than 5 frames will be deleted
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
        % Delete lost tracks.
        tracks = tracks(~lostInds); % delted tracks lose its place; the indices undergo a total change.
    end
    %%     createNewTracks();
    centrecoid = ccoid(unassignedDetections, :);%
    bboxes = bb(unassignedDetections, :);
    for i = 1:size(centrecoid, 1)
        centroid = centrecoid(i,:);
        bbox = bboxes(i, :);
        %             % Create a Kalman filter object----which is the unassigned
        %             % centroid
        kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
            centroid, [200, 50], [100, 25], 100);
        F_update = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1];
        Npop_particles = 600;
        Xstd_rgb = 30;
        Xstd_pos = 2;
        Xstd_vec = 5;
        particle=[];
        Xrgb_trgt = [255; 255; 255];
        % Create a new track.
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'kalmanFilter', kalmanFilter, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0,...
            'frequency',bbox(5),...
            'velo',0,....
            'particles',[]);
        Npix_resolution=bbox;
        % Add it to the array of tracks.
        tracks(end+1) = newTrack;
        tracks(end).particles = create_particles(Npix_resolution, Npop_particles);
        % Increment the next id.
        nextId = nextId + 1;
    end
    %%     displayTrackingResults();
    % % Showing Image
    show_particles(tracks, f);
    
    X_prev=show_state_estimated2(tracks, f,X_prev);
    % % % % Convert the frame and the mask to uint8 RGB.
    % %         mask = uint8(repmat(bf, [1, 1, 3])) .* 255;
    %
    %         minVisibleCount = 3;
    %         if ~isempty(tracks)
    %
    %             % Noisy detections tend to result in short-lived tracks.
    %             % Only display tracks that have been visible for more than
    %             % a minimum number of frames.
    %             reliableTrackInds = ...
    %                 [tracks(:).totalVisibleCount] > minVisibleCount;
    %             reliableTracks = tracks(reliableTrackInds);
    %
    %             % Display the objects. If an object has not been detected
    %             % in this frame, display its predicted bounding box.
    %             if ~isempty(reliableTracks)
    %                 % Get bounding boxes.
    %                 bboxes = cat(1, reliableTracks.bbox);
    %
    %                 % Get ids.
    %                 ids = int32([reliableTracks(:).id]);
    %
    %                 % Create labels for objects indicating the ones for
    %                 % which we display the predicted rather than the actual
    %                 % location.
    %                 labels = cellstr(int2str(ids'));
    %                 predictedTrackInds = ...
    %                     [reliableTracks(:).consecutiveInvisibleCount] > 0;
    %                 isPredicted = cell(size(labels));
    %                 isPredicted(predictedTrackInds) = {' predicted'};
    %                 labels = strcat(labels, isPredicted);
    %
    %                 % Draw the objects on the frame.
    %                 f = insertObjectAnnotation(f, 'rectangle', ...
    %                     bboxes(:,1:4), labels);%was org bboxes only
    %
    %                 % Draw the objects on the mask.
    %                 mask = insertObjectAnnotation(mask, 'rectangle', ...
    %                     bboxes(:,1:4), labels); %was org bboxes only
    %             end
    %         end
    % figure
    % imshow(mask),title(kk);
    % figure
    %  imshow(f),title(kk);
    % Video writing ---part 2
    %store annotated image
    %  filename = [sprintf('%05d',kk) '.png']
    %    fullname1 = fullfile(workingDir1,'images',filename);
    %    imwrite(f,fullname1);%changed f to bf
    %    fullname2= fullfile(workingDir2,'images',filename);
    %    imwrite(mask,fullname2);
    
end
%% Storm parameters
%prediction of next position of each bounding box
BB1=zeros(length(tracks),5);BB2=zeros(length(tracks),5);
PC=zeros(length(tracks),2);
for i = 1:length(tracks)
    BB1(i,:)= tracks(i).bbox;
    % Predict the current location of the track.
    predictedCentroid = predict(tracks(i).kalmanFilter);
    PC(i,:)=predictedCentroid;
    % Shift the bounding box so that its center is at
    % the predicted location.
    predictedCorner = int32(predictedCentroid) - int32(BB(3:4)) / 2;
    BB2(i,:)= [predictedCorner, BB(3:4),BB(5)];
    tracks(i).bbox=BB2(i,:);
end
%Velocity
mperpixel=4593;
timeLapseFactor=6000;
Frate=30;
%for video 2 the spatial resolution is 4.6 km
for i=1:length(tracks)
    disp('         ');
    disp('Track ID:');
    disp(tracks(i).id);
    disp('Predicted Centroid:');
    disp(PC(i,:));
    disp('Predicted Bounding Box:');
    disp(tracks(i).bbox);
    disp('Region of Risk:(Area in sq.km)');
    disp((tracks(i).bbox(3)).*(tracks(i).bbox(4))*(mperpixel/1000).^2);
    % disp('Predicted Velocity:(m/s)');
    % disp(vel(i));
    disp('Intensity of lightning strikes:');
    disp(tracks(i).frequency);
    disp('Expected frequency of strikes:)');
    disp(tracks(i).frequency/tracks(i).age);
end
% THE PREDICTORS
% avg frequency per track--frequency/age--done
% new locations predicted-- centroid and bounding boxes, done
% velocity--done
% confidence of prediction
% regions of risk--- same as bounding boxes--done
% intesnity of strom in the specified bounding box--- same as frequency
% % Video writing ---part 3
% imageNames = dir(fullfile(workingDir1,'images','*.png'));
% imageNames = {imageNames.name}';
% outputVideo = VideoWriter(fullfile(workingDir1,'mykvidframes.avi'));
% outputVideo.FrameRate = Frate;
% open(outputVideo)
% for ii = 1:length(imageNames)
%    img = imread(fullfile(workingDir1,'images',imageNames{ii}));
%    writeVideo(outputVideo,img)
% end
% close(outputVideo);
%
% imageNames = dir(fullfile(workingDir2,'images','*.png'));
% imageNames = {imageNames.name}';
% outputVideo = VideoWriter(fullfile(workingDir2,'mykvidmasks.avi'));
% outputVideo.FrameRate = Frate;
% open(outputVideo)
% for ii = 1:length(imageNames)
%    img = imread(fullfile(workingDir2,'images',imageNames{ii}));
%    writeVideo(outputVideo,img)
% end
% close(outputVideo);