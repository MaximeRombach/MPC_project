function save_plot(filename,save)
% For the saving to occur, the working directory has to be \common and
% not common\d_3_1 for example. Click on the little "+" next to the desired
% folder to stay in the directory
% If save == false in the main code no need of the note above
if save == true
    
    addpath('./figures')

    saveas(gcf, fullfile('figures/',filename), 'epsc')
    saveas(gcf, fullfile('figures/',[filename '.png']))

end

end