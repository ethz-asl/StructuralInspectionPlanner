%%  PLOT script for the Airplane2p5D solver

fname = 'airplane_solution'; fext = '.txt'; filename = [fname fext];
AirplaneSolution = load(filename);

h = figure;
subplot(3,2,[1 2 3 4])
plot3(AirplaneSolution(:,1),AirplaneSolution(:,2),AirplaneSolution(:,3),'bs-','LineWidth',2,'LineSmoothing','on','MarkerFaceColor','r','MarkerSize',4);
grid on; xlabel('x'); ylabel('y'); zlabel('z'); box on;
subplot(3,2,[5 6])
plot(AirplaneSolution(:,4),'ro-','LineWidth',2,'LineSmoothing','on','MarkerFaceColor','b','MarkerSize',4); 
grid on; xlabel('Samples'); ylabel('Yaw (rad)'); 
savefig(h,[fname '.fig'])

