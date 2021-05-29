function X = create_particles(Npix_resolution, Npop_particles)
%X1 and X2 are spacial coordinates of particles
 X2 = randi([Npix_resolution(1),Npix_resolution(1)+Npix_resolution(3)], 1, Npop_particles);
%  X2 = randi([Npix_resolution(1),Npix_resolution(1)+2], 1, Npop_particles);
 X1 = randi([Npix_resolution(2),Npix_resolution(2)+Npix_resolution(4)], 1, Npop_particles);%INTERCHANGED x1 AND X2
%  X1 = randi([Npix_resolution(2),Npix_resolution(2)+2], 1, Npop_particles)
 X3 = zeros(2, Npop_particles);
e='i am here'
X = [X1; X2; X3];