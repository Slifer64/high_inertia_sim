

ax = gco;

childs = allchild(ax);

pos_line = childs(2);
neg_line = childs(3);

n_data = length(pos_line.XData);

x = 0.16;
x_new = 0;

x_data = pos_line.XData;

for i=1:n_data
   if (x_data(i) >= x); break; end 
end

x_data(i:end) = x_data(i:end) + (x_new - x_data(i));

pos_line.XData = x_data;
neg_line.XData = x_data;

pos_line.YData = pos_line.YData(i:end);
neg_line.YData = neg_line.YData(i:end);

drawnow();
