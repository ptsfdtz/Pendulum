function dp_animation(t,x,theta1,theta2)
% =========================================================
% 二级倒立摆实时动画
% =========================================================

persistent fig ax
persistent cart
persistent rod1 rod2
persistent joint0 joint1 tip
persistent stateText

P = dp_config();

L1 = P.L1;
L2 = P.L2;

cw = P.cartWidth;
ch = P.cartHeight;

%% =========================================================
% 几何位置
%% =========================================================

% 小车转轴
x0 = x;
y0 = 0;

% 动力学模型定义正角向 -x（左），与论文/LQR 的输入符号一致。
% 实物编码器向 +x 计数增加，因此硬件换算层会对摆角取负号。
% 一级摆末端 = 二级摆转轴
x1 = x0 - L1*sin(theta1);
y1 = y0 + L1*cos(theta1);

% 二级摆末端
x2 = x1 - L2*sin(theta2);
y2 = y1 + L2*cos(theta2);

%% =========================================================
% 第一次运行创建窗口
%% =========================================================

if isempty(fig) || ~isgraphics(fig)

    fig = figure( ...
        'Name','二级倒立摆实时仿真', ...
        'NumberTitle','off', ...
        'Position',P.figurePosition, ...
        'Color','w');

    ax = axes( ...
        'Parent',fig, ...
        'Position',[0.07 0.12 0.90 0.80]);

    hold(ax,'on');
    grid(ax,'on');

    xlabel(ax,'Position X / m');
    ylabel(ax,'Height Y / m');

    title(ax,'Double Inverted Pendulum');

    %% 显示范围

    xlim(ax,[-P.viewX P.viewX]);
    ylim(ax,[P.viewYMin P.viewYMax]);

    % 让画布更宽
    pbaspect(ax,[1.8 1 1]);

    %% =====================================================
    % 轨道
    %% =====================================================

    plot(ax, ...
        [-P.x_limit P.x_limit], ...
        [-ch -ch], ...
        'k-', ...
        'LineWidth',5);

    %% 边界

    plot(ax, ...
        [-P.x_limit -P.x_limit], ...
        [-0.14 0.15], ...
        'r--', ...
        'LineWidth',1.5);

    plot(ax, ...
        [P.x_limit P.x_limit], ...
        [-0.14 0.15], ...
        'r--', ...
        'LineWidth',1.5);

    text(ax, ...
        -P.x_limit,-0.17, ...
        sprintf('-%.2f m',P.x_limit), ...
        'HorizontalAlignment','center');

    text(ax, ...
        P.x_limit,-0.17, ...
        sprintf('+%.2f m',P.x_limit), ...
        'HorizontalAlignment','center');

    %% =====================================================
    % 小车
    %% =====================================================

    cart = rectangle( ...
        'Parent',ax, ...
        'Position',[x0-cw/2,-ch,cw,ch], ...
        'FaceColor',[0.80 0.80 0.80], ...
        'EdgeColor','k', ...
        'LineWidth',2);

    %% 一级摆

    rod1 = plot(ax, ...
        [x0 x1], ...
        [y0 y1], ...
        'LineWidth',5);

    %% 二级摆

    rod2 = plot(ax, ...
        [x1 x2], ...
        [y1 y2], ...
        'LineWidth',5);

    %% 关节

    joint0 = plot(ax,x0,y0,'ko', ...
        'MarkerSize',9, ...
        'MarkerFaceColor','w', ...
        'LineWidth',2);

    joint1 = plot(ax,x1,y1,'ko', ...
        'MarkerSize',9, ...
        'MarkerFaceColor','w', ...
        'LineWidth',2);

    tip = plot(ax,x2,y2,'ko', ...
        'MarkerSize',7, ...
        'MarkerFaceColor','k');

    %% 状态显示

    stateText = text(ax, ...
        0.01,0.98,'', ...
        'Units','normalized', ...
        'VerticalAlignment','top', ...
        'FontSize',11);

end

%% =========================================================
% 更新小车
%% =========================================================

set(cart, ...
    'Position', ...
    [x0-cw/2,-ch,cw,ch]);

%% 一级摆

set(rod1, ...
    'XData',[x0 x1], ...
    'YData',[y0 y1]);

%% 二级摆

set(rod2, ...
    'XData',[x1 x2], ...
    'YData',[y1 y2]);

%% 关节

set(joint0, ...
    'XData',x0, ...
    'YData',y0);

set(joint1, ...
    'XData',x1, ...
    'YData',y1);

set(tip, ...
    'XData',x2, ...
    'YData',y2);

%% =========================================================
% 越界报警
%% =========================================================

if abs(x) >= P.x_limit

    set(cart, ...
        'FaceColor',[1.0 0.30 0.30]);

else

    set(cart, ...
        'FaceColor',[0.80 0.80 0.80]);

end

%% =========================================================
% 角度显示
%% =========================================================

a1 = atan2(sin(theta1),cos(theta1));
a2 = atan2(sin(theta2),cos(theta2));

a1 = a1*180/pi;
a2 = a2*180/pi;

set(stateText, ...
    'String',sprintf( ...
    't = %.2f s     x = %.3f m     theta1 = %.1f deg     theta2 = %.1f deg', ...
    t,x,a1,a2));

drawnow limitrate;

end
