clear all
clc
filename = 'csv1.dat';
M = csvread(filename);
N = M;
circ = 0.2*pi;
[a ~] = size(M);
M(:,1) = M(:,1)-N(1,1);
xy(a,1:2) = [0,0];
for k = 2:a
    xy(k,1:2) = [(xy(k-1,1)+((M(k,4)-M(k-1,4))*circ*cosd(M(k,1)))),(xy(k-1,2)+((M(k,4)-M(k-1,4))*circ*sind(M(k,1))))];
end
%ml = plot(xy(:,1),xy(:,2));
%set(ml,'Color','blue','LineWidth',0.0001,'MarkerSize',5,'MarkerEdgeColor','blue','MarkerFaceColor','blue')
hold on
fr(a,1:2) = [0,0];
for k = 1:a
   fr(k,1:2) = [xy(k,1)+M(k,2)*circ*cosd(M(k,1)),xy(k,2)+M(k,2)*circ*sin(M(k,1))]; 
end
p = plot(fr(:,1),fr(:,2));
sr(a,1:2) = [0,0];
for k = 1:a
   sr(k,1:2) = [xy(k,1)+M(k,2)*circ*cosd(90+M(k,1)),xy(k,2)+M(k,2)*circ*sin(90+M(k,1))]; 
end
c = plot(sr(:,1),sr(:,2));
set(p,'Color','red','LineWidth',0.0001,'MarkerSize',5,'MarkerEdgeColor','red','MarkerFaceColor','red')
set(c,'Color','green','LineWidth',0.0001,'MarkerSize',5,'MarkerEdgeColor','green','MarkerFaceColor','green')