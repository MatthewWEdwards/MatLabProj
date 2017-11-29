%%%%%%%%
%
% An example from MATLAB computer vision toolbox
% Adapted by Cooper Raterink on 11.28
%
%%%%%%%%
function [showFrameOnL, showFrameOnR, hFig] = SideBySideVideo(mainTitle, leftTitle, rightTitle, btnTitle, toggleBtnCallback)
    [hFig, hAxes] = createFigureAndAxes(mainTitle, leftTitle, rightTitle);
    insertButtons(hFig,btnTitle, toggleBtnCallback);
    showFrameOnL = @(frame) showFrameOnAxis(hAxes.axis1, frame);
    showFrameOnR = @(frame) showFrameOnAxis(hAxes.axis2, frame);
end

function [hFig, hAxes] = createFigureAndAxes(mainTitle, leftTitle, rightTitle)

        % Close figure opened by last run
%         figTag = 'CVST_VideoOnAxis_9804532';
%         close(findobj('tag',figTag));

        % Create new figure
        hFig = figure('numbertitle', 'off', ...
               'name', mainTitle, ...
               'menubar','none', ...
               'toolbar','none', ...
               'resize', 'on', ...
               'renderer','painters', ...
               'position',[680 678 480 240]);
        figureFullScreen(hFig);

        % Create axes and titles
        hAxes.axis1 = createPanelAxisTitle(hFig,[0.1 0.2 0.36 0.6],leftTitle); % [X Y W H]
        hAxes.axis2 = createPanelAxisTitle(hFig,[0.5 0.2 0.36 0.6],rightTitle);
end
function hAxis = createPanelAxisTitle(hFig, pos, axisTitle)

    % Create panel
    hPanel = uipanel('parent',hFig,'Position',pos,'Units','Normalized');

    % Create axis
    hAxis = axes('position',[0 0 1 1],'Parent',hPanel);
    hAxis.XTick = [];
    hAxis.YTick = [];
    hAxis.XColor = [1 1 1];
    hAxis.YColor = [1 1 1];
    % Set video title using uicontrol. uicontrol is used so that text
    % can be positioned in the context of the figure, not the axis.
    titlePos = [pos(1)+0.02 pos(2)+pos(3)+0.3 0.3 0.07];
    uicontrol('style','text',...
        'String', axisTitle,...
        'Units','Normalized',...
        'Parent',hFig,'Position', titlePos,...
        'BackgroundColor',hFig.Color);
end
function insertButtons(hFig,btnTitle, cb)
    % toggle button with text btnTitlw
    uicontrol(hFig,'unit','pixel','style','pushbutton','string',btnTitle,...
            'position',[600 100 75 25], 'tag','PBButton123','callback',...
            cb);
end
