function save_plots(figures, names, PATH)

n = length(figures);

for i = 1:n
    figure = figures(i);
    set(figure, 'Renderer', 'Painters');
    
    saveas(f,[PATH '\' names(i) '.eps'],'epsc');
    
end

