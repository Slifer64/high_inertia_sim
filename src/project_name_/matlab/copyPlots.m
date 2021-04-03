
n_childs = length(from_ax.Children);
for i=1:n_childs
    [num2str(i) ' / ' num2str(n_childs)]
    copyobj(from_ax.Children(i), to_ax)
end

