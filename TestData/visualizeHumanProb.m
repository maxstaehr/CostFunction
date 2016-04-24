% sceneNumber = 41;

str = strcat('sampleHumanState_scene', num2str(sceneNumber), '.bin');
fileId = fopen(str);


nPoints = fread(fileId, 1, 'int32');
R = zeros(nPoints, 2,2);
V = zeros(nPoints, 2);
P = zeros(nPoints,1);
for i=1:nPoints
    for r=1:2
        for c=1:2                    
            R(i,r,c) = fread(fileId, 1, 'single');                    
        end
    end
end
V(:,1) = fread(fileId, nPoints, 'single');
V(:,2) = fread(fileId, nPoints, 'single');
P(:)  = fread(fileId, nPoints, 'double');
maxP =  fread(fileId, 1, 'double');
fclose(fileId);
disp(maxP);


Z = linkage(V,'ward','euclidean','savememory','off');
c = cluster(Z,'cutoff',0.1);
Data = zeros(max(c), 3);
for i=1:max(c)
    C = V(c==i,:);
    Prob = P(c==i,:);
    Data(i,:) = [C(1,:) max(Prob)];
end
X = Data(:,1);
Y = Data(:,2);

Z = linkage(X,'ward','euclidean','savememory','off');
cx = cluster(Z,'cutoff',0.1);
NClusterX = max(cx);
minX = min(X);
maxX = max(X);
dx = (maxX-minX)/(NClusterX-1);


Z = linkage(Y,'ward','euclidean','savememory','off');
cy = cluster(Z,'cutoff',0.1);
NClusterY = max(cy);
minY = min(Y);
maxY = max(Y);
dy = (maxY-minY)/(NClusterY-1);

[Xgrid Ygrid] = meshgrid(minX:dx:maxX, minY:dy:maxY);
Z = zeros(NClusterY, NClusterX);
for i=1:numel(Xgrid)
    
    IDX = knnsearch(Data(:,1:2), [Xgrid(i) Ygrid(i)]);    
    Z(i) = Data(IDX,3);
    
    
end
figure; hold on;
surf(Xgrid,Ygrid,Z);

