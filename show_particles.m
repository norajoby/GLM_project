function show_particles(tracks, Y_k)
figure(1)
image(Y_k)
for i=1:length(tracks)
title('+++ Showing Particles +++')
%[tracks(i).particles(1,:); tracks(i).particles(2,:)]

hold on
plot(tracks(i).particles(2,:), tracks(i).particles(1,:), 'b*');
end

drawnow