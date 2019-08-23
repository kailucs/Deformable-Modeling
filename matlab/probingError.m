function e = probingError(Model,linearData,weight,pcd)
%Each model is a column vector of coefficients
%this is the current estimate of point model
totalFeatures = size(Model,1);
Model1 = Model(1:totalFeatures/4);
Model2 = Model(totalFeatures/4+1:totalFeatures/2);
Model3 = Model(totalFeatures/2+1:3*totalFeatures/4);
Model4 = Model(3*totalFeatures/4+1:totalFeatures);
e = 0;

%% Loop over the probing locations
for iter = 1:1:linearData.NofPoints
    lineStart = linearData.lineStarts(iter,:);
    lineEnd = linearData.lineStarts(iter,:);
    lineNormal = linearData.lineNormals(iter,:);
    torqueAxis = linearData.lineTorqueAxes(iter,:);
    lineCenter = (lineEnd-lineStart)*0.5+lineStart;
    discretization = 0.001;
    N = 1 + round(norm(lineStart- LineEnd)/discretization);
    s = linespace(0,1,N);
    %% first project the pcd to the plane of the line 
    %Note we need to careful if 2 surfaces will be projected onto the
    %plane....for now it's fine...
    %also do something naive to restrict the amount of pts..

    projectedPcd = zeros(size(pcd,1),3); %preallocate and cut later
    projectedPcdIdxList = zeros(size(pcd,1),3);%Idx in the original pcd 
    NofProjectedPoints = 0;
    for i=1:1:size(pcd,1)
        p = pcd(i,:);
        projectedPt = p-lineStart-dot((p-lineStart),lineNormal)*lineNormal;
        if -0.0001 < dot(projectedPt,lineStart-lineEnd) < 1.0001 && ...
                dot((p-lineStart),lineNormal) > 0 %make sure point is between
            %the two ends and is "above" the line.
            NofProjectedPoints = NofProjectedPoints + 1;
            projectedPcd(NofProjectedPoints,:) = projectedPt;
            projectedPcdIdxList(NofProjectedPoints,:) = i;
        end
    end
    projectedPcd(NofProjectedPoints:size(pcd,1),:) = [];%cut the excessive space

    %% calculate the norminal displacements
    surfacePts = zeros(NofProjectedPoints,3); %These are the surface pts that will be displaced.
    surfacePtsNormal = zeros(NofProjectedPoints,3);
    nominalD = zeros(N,1); %Column Vector..
    %part of the probe not in contact with the object...
    NofN = 1;
    for i=1:1:NofProjectedPoints
        linePt =(lineEnd-lineStart)*s(i)+lineStart; %the point on the line, projected 
        %onto the line
        Idx = knnsearch(projectedPcd,linePt,NofN);
        %%%We might end up having duplicated pts...
        %%%We should make sure that the discretization is not too fine..
        %%%or should average a few neighbors
        surfacePt = [0 0 0];
        surfacePtNormal = [0 0 0];
        for j=1:1:NofN
            surfacePt = surfacePt + pcd(projectedPcdIdxList(Idx(j)),:);
            normal = normal + pcdNormal(Idx(j),:);
        end
        surfacePt = surfacePt/NofN;
        normal = normalize(surfacePtNormal/NofN);
        nominalDisp = dot(surfacePt-lineStart-projectPts(Idx,:),normal);
        surfacePts(i,:) = surfacePt;%position in the global frame..
        nominalD(i) = nominalDisp;
    end
    %% Calculate Actual Displacements
    K = zeros(N,N);
    for i = 1:1:N 
        for j = 1:1:N
            K(i,j) = linearKernel(surfacePts(i),surfacePts(j));
        end
    end
    det(K)
    actualD = K\nominalD;
    
    %% Get the point model    
    totalF = [0 0 0]; %row vector
    totalTorque = 0; %units:Nm
    for i=1:1:N
       feature = surfacePts(i,1:6);%Get the features of this surface pt
       %query for the coefficients..
       C1 = dot(Model1,learningKernelFunc(feature)); 
       C2 = dot(Model2,learningKernelFunc(feature)); 
       C3 = dot(Model3,learningKernelFunc(feature)); 
       C4 = dot(Model4,learningKernelFunc(feature));
       d = actualD(i,1); %calculated actual displacements 
       force = C1*d+C2*d^2+C3*d^3+C4*d^4;
       totalF = totalF + force*surfacePtsNormal(i,:);
       linePt =(lineEnd-lineStart)*s(i)+lineStart;
       %Use the point on the line to calculate torque..
       torque = dot(cross(linePt-lineCenter,force),torqueAxis);
       totalTorque = totalTorque + torque;
    end
    forceMagnitude = dot(totalF,lineNormal);
    e = e + (forceMagnitude-linearData(i,n))^2 + ...
        (totalTorque-linearData(i,n))^2 ; %modify this line
end
e = e + weight*norm(Model)^2;
end


