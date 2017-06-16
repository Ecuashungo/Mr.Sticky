
% Guide
% d     : select new bottle in same image
% w & r : change current bottle of the current image (previous & next)
% e     : delete current bottle rectangle
% s & f : change image (previous & next)



clearvars; close all; clc;

[fn0,pn0]=uigetfile({'*.jpg'},'Select image (*.jpg)');
listing = dir(pn0);

i = 1;
name = getfield(listing,{i}, 'name');
while name(1)=='.'
    i = i+1;
    name = getfield(listing,{i}, 'name');
end
listing = listing(i:end);

N = size(listing,1);

h = figure('Name','Bottle selection');
hold on


%% rectangle selection
for i = 1:size(listing,1)
    memory(i).checked = 0;
    memory(i).rectangle = [];
end
currentFrame = 1;
currentBottle = 0;
continueSelection = 1;

I = imread([getfield(listing,{i}, 'folder'),'/',getfield(listing,{currentFrame}, 'name')]);
dim = size(I);
dim = flip(dim(1:2),2);

while continueSelection
        
    I = imread([getfield(listing,{i}, 'folder'),'/',getfield(listing,{currentFrame}, 'name')]);            
    imshow(I);
    
    showRectangles(memory(currentFrame).rectangle,currentBottle);
    
    k=0;
    while ~k
        k = waitforbuttonpress;
        currkey = get(gcf,'currentcharacter');
        
        if strcmp(currkey,'d')
            k = 1;
            disp('select more btl');
            currentBottle = currentBottle+1;
            memory(currentFrame).rectangle(currentBottle,:) = adjustSize(getrect,dim);
            
            while ( (memory(currentFrame).rectangle(currentBottle,2)==0) | (memory(currentFrame).rectangle(currentBottle,3)==0) )
                memory(currentFrame).rectangle(currentBottle,:) = adjustSize(getrect,dim);
            end
            
        elseif strcmp(currkey,'f')
            k = 1;
            disp('next frame');
            memory(currentFrame).checked = 1;
            currentFrame = currentFrame+1;
            currentBottle = 0;
            
        elseif strcmp(currkey,'s')
            k = 1;
            disp('previous frame');
            memory(currentFrame).checked = 1;
            currentFrame = currentFrame-1;
            currentBottle = 0;
        
        elseif strcmp(currkey,'e')
            k = 1;
            disp('remove current btl');
            if (currentBottle ~= 0)
                memory(currentFrame).rectangle(currentBottle,:) = [];
                currentBottle = currentBottle-1;
            end
            
        elseif strcmp(currkey,'r')
            k = 1;
            disp('next btl as current');
            currentBottle = currentBottle+1;
            
        elseif strcmp(currkey,'w')
            k = 1;
            disp('previous btl as current');
            currentBottle = currentBottle-1;
            
        elseif strcmp(currkey,'q')
            k = 1;
            disp('quit');
            memory(currentFrame).checked = 1;
            continueSelection = false;
            
        else
            k=0;
        end
    end
    
    % check currentFrame out range
    
    if (currentFrame<1)
        currentFrame = 1;
    elseif (currentFrame>N)
        currentFrame = N;
    end
    
    
    % check currentBottle out range
    
    if (currentBottle<1)
        if isempty(memory(currentFrame).rectangle)
            currentBottle = 0; % there is no bottles
        else 
            currentBottle = size(memory(currentFrame).rectangle,1); % there is bottles at leat 1
        end
    elseif currentBottle>size(memory(currentFrame).rectangle,1)
        currentBottle=1;
    end
    
    
end

close all;

%% insert into txt file

fileID = fopen([getfield(listing,{1}, 'folder'),'/data.txt'],'w');
sp = ' '; % standard space

newdir = [getfield(listing,{1}, 'folder'),'/positive'];
mkdir(newdir)

newdir = [getfield(listing,{1}, 'folder'),'/negative'];
mkdir(newdir)

newdir = [getfield(listing,{1}, 'folder'),'/noChecked'];
mkdir(newdir)

for i = 1:N
    if (memory(i).checked)
        if (~isempty(memory(i).rectangle))
            nb_bottle = size(memory(i).rectangle,1);
            rectText = '';
            for j = 1:nb_bottle
                rect = memory(i).rectangle(j,:)+[-1 -1 0 0];
                rectText = [rectText,sp,num2str(round(rect(1))),sp,num2str(round(rect(2))),sp,num2str(round(rect(3))),sp,num2str(round(rect(4)))];
            end
            fprintf(fileID, ['positive/',getfield(listing,{i}, 'name'),sp,num2str(nb_bottle),sp,rectText,'\n']);
            copyfile([getfield(listing,{i}, 'folder'),'/',getfield(listing,{i}, 'name')],[getfield(listing,{i}, 'folder'),'/positive/',getfield(listing,{i}, 'name')])
        else
            copyfile([getfield(listing,{i}, 'folder'),'/',getfield(listing,{i}, 'name')],[getfield(listing,{i}, 'folder'),'/negative/',getfield(listing,{i}, 'name')])
        end
    else
        copyfile([getfield(listing,{i}, 'folder'),'/',getfield(listing,{i}, 'name')],[getfield(listing,{i}, 'folder'),'/noChecked/',getfield(listing,{i}, 'name')])
    end
end
fclose(fileID);