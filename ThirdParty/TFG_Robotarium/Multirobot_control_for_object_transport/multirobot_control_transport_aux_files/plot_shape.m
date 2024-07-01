function plot_shape(shape, edges, contour, color)

%Plots a mesh: the set of point positions, the mesh edges between them,
%and the contour (if given)

if size(shape,2)==1 %reshape to suitable size
    
    shape=reshape(shape,3,size(shape,1)/3);

end

hold on
for i=1:size(edges,1)
    
    plot3([shape(1,edges(i,1)) shape(1,edges(i,2))], [shape(2,edges(i,1)) shape(2,edges(i,2))], ...
    [shape(3,edges(i,1)) shape(3,edges(i,2))],'color',color) 

end

if size(contour,2)==1
    contour2=[];
    for i=1:size(contour,1)-1
        contour2=[contour2;[contour(i) contour(i+1)]];        
    end
else
    contour2=contour;
end

for i=1:length(contour2)
    
    plot3([shape(1,contour2(i,1)) shape(1,contour2(i,2))], [shape(2,contour2(i,1)) shape(2,contour2(i,2))], ...
    [shape(3,contour2(i,1)) shape(3,contour2(i,2))],'color',color) 

end