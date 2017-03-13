x = load("new.dat");
N = length(x);
dQG = x(:,2)/1000;
qA = atan2((x(:,5)-2048),-(x(:,4)-2048));
xH = [qA(1) 0]; 
K = [0.000003 0.0000002]'*0.5;

for i = 1: N-1
  temp = [1,0.01;0, 1]*xH(i,:)' + [0.01;0]*dQG(i);
  temp = temp - K*(xH(i,1) - qA(i));
  xH(i+1,:) = temp';
end
