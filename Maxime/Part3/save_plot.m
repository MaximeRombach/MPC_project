function save_plot(filename,plot)

if plot == true

    saveas(gcf, fullfile('figures/',filename), 'epsc')
    saveas(gcf, fullfile('figures/',[filename '.png']))

end

end