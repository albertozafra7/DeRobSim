clear all

p_m=[2 3 4]' %Punto en matlab

T=eye(4)
% R=Rgiros([0 0 -90])*Rgiros([0 -90 0])
R=eul2rotm([0 0 deg2rad(-90)])*eul2rotm([0 deg2rad(-90) 0])


T(1:3,1:3)=R

T(3,1)=-T(3,1)

p_u = T*[p_m;1] %Punto en unity