function X_prev=show_state_estimated2(tracks, Y_k,X_prev)
% for k=1:4:(4*noc)-3
% X_mean(k) = mean(X((k:k+1),:), 2);
% en
hold on
title('+++ Showing mean value +++')
for i=1:length(tracks)
 X_mean = mean(tracks(i).particles, 2);
 X_mean(3,:)=tracks(i).id;
 plot(X_mean(2,:), X_mean(1,:), 'h', 'MarkerSize', 16, 'MarkerEdgeColor', 'y', 'MarkerFaceColor', 'y');
end
%Velocity
mperpixel=4593;
timeLapseFactor=6000;
Frate=30;
diff=[];

for j=1: size(X_mean,1)
for ii=1:size(X_prev,1)
if X_mean(3,j)==X_prev(3,ii)
 diff=[diff,X_mean(2,j)-X_prev(2,ii); X_mean(1,j)-X_prev(1,ii)];
 dif2=diff.*diff;
 dis=(dif2(1,:)+dif2(2,:)).^(1/2);
vel=dis*mperpixel*Frate/timeLapseFactor;
disp(vel);
end
end
end
X_prev=X_mean;
hold off

drawnow