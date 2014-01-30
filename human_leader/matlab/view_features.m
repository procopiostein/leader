%visualize a typical dataset (expects 8 features)

function [] = crop_features(in_data)

% Show the data
H = figure;
set(H,'defaultlinelinewidth',3);
set(H,'defaultaxeslinewidth',1.5);
set(H,'defaulttextfontsize',12);
set(H,'defaultaxesfontsize',12);
hold on, grid on; title('Cropped Fatures');
set(gcf,'position',[200 200 800 300]);

plot(in_data(:,1),in_data(:,3),'bo');
plot(in_data(:,1),in_data(:,4),'co');
plot(in_data(:,1),in_data(:,5),'ko');
plot(in_data(:,1),in_data(:,6),'go');
plot(in_data(:,1),in_data(:,7),'yo');
plot(in_data(:,1),in_data(:,8),'mo');
plot(in_data(:,1),in_data(:,9),'o','color',[0.5 0 0]);
plot(in_data(:,1),in_data(:,10),'o','color',[0.5 0.5 0]);


legend('velocity (m/s)','lateral disp. (m)',...
    'relative head. (rad)','angle (rad)','distance (m)',...
    'relative vel. x (m/s)','relative vel. y (m/s)','sagital disp. (m)',...
    'location','eastOutside');
xlabel('t(s)');
