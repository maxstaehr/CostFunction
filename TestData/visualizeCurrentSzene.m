clear all;
sceneNumber = 0;
str = strcat('currentSzene_scene', num2str(sceneNumber), '.bin');
kinChainFileName = strcat('currentSzene_scene', num2str(sceneNumber), '.bin');
cameraSamplePointsFileName = strcat('currentSzeneSamplePoints_scene', num2str(sceneNumber), '.bin');
cameraSetupFileName = strcat('cameraSetup_scene', num2str(sceneNumber), '.bin');
clusterFileName = strcat('cluster_scene', num2str(sceneNumber), '.bin');


fileId = fopen(kinChainFileName);
nVertices = fread(fileId, 1, 'int32');
nFaces = fread(fileId, 1, 'int32');
nBB = fread(fileId, 1, 'int32');


V = zeros(3,nVertices);
F = zeros(3,nFaces);
bbH = zeros(nBB, 4, 4);
bbDim = zeros(nBB, 3);


V(1,:) = fread(fileId,  nVertices, 'single');
V(2,:) = fread(fileId,  nVertices, 'single');
V(3,:) = fread(fileId,  nVertices, 'single');

F(1,:) = fread(fileId,  nFaces, 'int32');
F(2,:) = fread(fileId,  nFaces, 'int32');
F(3,:) = fread(fileId,  nFaces, 'int32');

for j=1:size(bbH,1)
    for r=1:4
        for c=1:4                    
            bbH(j,r,c) = fread(fileId, 1, 'single');                    
        end
    end
end  

for j=1:size(bbDim,1)
    for r=1:3                        
        bbDim(j,r) = fread(fileId, 1, 'single');                            
    end
end





fclose(fileId);
F = F +1;

F = F';
V = V';


fileId = fopen(cameraSamplePointsFileName);
nPoints = fread(fileId, 1, 'int32');
S = zeros(nPoints, 4, 4);
for j=1:size(S,1)
    for r=1:4
        for c=1:4                    
            S(j,r,c) = fread(fileId, 1, 'single');                    
        end
    end
end            
fclose(fileId);
SPCL = zeros(nPoints, 3);
Zdir = [0 0 1 0]';
H = eye(4);
UVX = zeros(size(S,1), 3);
for j=1:size(S,1)
    SPCL(j,1) = S(j,1,4);
    SPCL(j,2) = S(j,2,4);
    SPCL(j,3) = S(j,3,4);
    H(:,:) = S(j,:,:);
    dir = H*Zdir;
    dirtrans = dir';
    UVX(j,:) = dir(1:3);
end


close all;
figure;
hold on;
grid on;
axis equal;
xlim([-5 5]);
ylim([-5 5]);
trisurf(F,V(:,1),V(:,2),V(:,3));
quiver3(SPCL(:,1),SPCL(:,2),SPCL(:,3), UVX(:,1), UVX(:,2), UVX(:,3));
plot3(SPCL(:,1),SPCL(:,2),SPCL(:,3), '.r');
axis equal;

lBB = cell(nBB, 1);
H = zeros(4);
for i=1:nBB
    H(:,:) = bbH(i,:,:);
    lBB{i} = BoundingBox(H, bbDim(i,1), bbDim(i,2), bbDim(i,3));
    lBB{i}.plotBox('r');
end


fileId = fopen(cameraSetupFileName);
nCamera = fread(fileId, 1, 'int32');
for i=1:nCamera
    H = zeros(4);
    for r=1:4
        for c=1:4                    
            H(r,c) = fread(fileId, 1, 'single');                    
        end
    end
    nRays = fread(fileId, 1, 'int32');
    nssRays = fread(fileId, 1, 'int32');
    
    R = zeros(nRays, 7);
    for j=1:numcols(R)
        R(:,j) = fread(fileId, nRays, 'single');
    end
    SSR = zeros(nssRays, 7);
    for j=1:numcols(SSR)
        SSR(:,j) = fread(fileId, nssRays, 'single');
    end
    
    
    nVertices = fread(fileId, 1, 'int32');
    nFaces = fread(fileId, 1, 'int32');


    V = zeros(3,nVertices);
    F = zeros(3,nFaces);

    V(1,:) = fread(fileId,  nVertices, 'single');
    V(2,:) = fread(fileId,  nVertices, 'single');
    V(3,:) = fread(fileId,  nVertices, 'single');

    F(1,:) = fread(fileId,  nFaces, 'int32');
    F(2,:) = fread(fileId,  nFaces, 'int32');
    F(3,:) = fread(fileId,  nFaces, 'int32');

    F = F +1;
    F = F';
    V = V';
    
    Hbb = zeros(4);
    DIM = zeros(3,1);
    for r=1:4
        for c=1:4                    
            Hbb(r,c) = fread(fileId, 1, 'single');                    
        end
    end
    DIM(:,1) = fread(fileId,  3, 'single');   
    
    %%vis data
    Rvis = R(1:10:end, :);
    O = repmat(H(1:3,4)', numrows(Rvis), 1);
    quiver3(O(:,1),O(:,2),O(:,3), Rvis(:,1), Rvis(:,2), Rvis(:,3));
    Rvis = R;
    Rvis(Rvis > 10) = NaN;
    plot3(R(:,5),R(:,6),R(:,7), '.r');
    
end
fclose(fileId);

%%
%
fileId = fopen(clusterFileName);
nPoints = fread(fileId, 1, 'int32');
Centriod = zeros(1,3);
C = zeros(nPoints, 3);
if nPoints > 0

    C(:,1) = fread(fileId, nPoints, 'single');
    C(:,2) = fread(fileId, nPoints, 'single');
    C(:,3) = fread(fileId, nPoints, 'single');
end
Centriod(1,:) = fread(fileId, 3, 'single');
fclose(fileId);
plot3(C(:,1),C(:,2),C(:,3), 'og');
plot3(Centriod(:,1),Centriod(:,2),Centriod(:,3), 'ok', 'MarkerSize', 10);
visualizeHumanProb;