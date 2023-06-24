import os.path

import wx

import AStarSolver
from BenchmarkGenerator import generator
from GridEnvironment import GridEnv
from PlotDraw import draw_origin_grid_plot, draw_grid_plot
from ProblemParser import grid_parameters

APP_ICON = 'icon/logo.ico'


class CustomizeFrame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, u'自定义生成布线问题', size=(370, 280),
                          style=wx.MINIMIZE_BOX | wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX)
        self.SetBackgroundColour(wx.Colour(240, 240, 240))
        self.Center()

        icon = wx.Icon(APP_ICON, wx.BITMAP_TYPE_ICO)
        self.SetIcon(icon)

        wx.StaticText(self, -1, u'自定义文件路径：', pos=(40, 50), size=(100, -1), style=wx.ALIGN_RIGHT)
        # self.tip = wx.StaticText(self, -1, u'', pos=(145, 110), size=(150, -1), style=wx.ST_NO_AUTORESIZE)

        self.tc1 = wx.TextCtrl(self, -1, '', pos=(145, 50), size=(150, -1), name='TC01', style=wx.TE_CENTER)

        self.button1 = wx.Button(self, -1, u'生成引脚图', pos=(50, 130))
        self.button1.Bind(wx.EVT_BUTTON, self.OnButtonClick1)
        self.button2 = wx.Button(self, -1, u'开始布线', pos=(190, 130))
        self.button2.Bind(wx.EVT_BUTTON, self.OnButtonClick2)

    def OnButtonClick1(self, event):
        filename = self.tc1.GetValue()
        if os.path.isfile(filename):
            gp = grid_parameters(filename)
            env = GridEnv(gp)
            draw_origin_grid_plot(env)
        else:
            er_msg = wx.MessageDialog(None, "文件路径不存在", "错误信息提示", wx.YES_DEFAULT | wx.ICON_ERROR)
            if er_msg.ShowModal() == wx.ID_YES:  # 如果点击了提示框的确定按钮
                er_msg.Destroy()  # 则关闭提示框

    def OnButtonClick2(self, event):
        filename = self.tc1.GetValue()
        if os.path.isfile(filename):
            gp = grid_parameters(filename)
            gridEnv = GridEnv(gp)
            while gridEnv.episode < 5:
                gridEnv.reset()
                if gridEnv.episode == 5:
                    draw_grid_plot(gridEnv)
                if gridEnv.episode == 5:
                    break
                gridEnv.breakup()
                route, cost = AStarSolver.a_star_route(gridEnv.init_pos, gridEnv.goal_pos,
                                                       gridEnv.occupied_coord, gridEnv.netPinSet, gp['gridSize'])
                # gridEnv.route = route
                # gridEnv.set_route(route)
                # gridEnv.cost = cost
                gridEnv.update(route, cost)
        else:
            er_msg = wx.MessageDialog(None, "文件路径不存在", "错误信息提示", wx.YES_DEFAULT | wx.ICON_ERROR)
            if er_msg.ShowModal() == wx.ID_YES:  # 如果点击了提示框的确定按钮
                er_msg.Destroy()  # 则关闭提示框


class RandomFrame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, u'随机生成布线问题', size=(370, 280),
                          style=wx.MINIMIZE_BOX | wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX)
        self.SetBackgroundColour(wx.Colour(240, 240, 240))
        self.Center()

        icon = wx.Icon(APP_ICON, wx.BITMAP_TYPE_ICO)
        self.SetIcon(icon)

        self.t1 = wx.StaticText(self, -1, u'网格水平范围：\ndefault=32  ', pos=(0, 30), size=(100, -1), style=wx.ALIGN_RIGHT)
        self.t2 = wx.StaticText(self, -1, u'网格垂直范围：\ndefault=32  ', pos=(170, 30), size=(100, -1), style=wx.ALIGN_RIGHT)
        self.t3 = wx.StaticText(self, -1, u'布线层数：\ndefault=2  ', pos=(0, 70), size=(100, -1), style=wx.ALIGN_RIGHT)
        self.t4 = wx.StaticText(self, -1, u'网络数量：\ndefault=3  ', pos=(170, 70), size=(100, -1), style=wx.ALIGN_RIGHT)
        self.t5 = wx.StaticText(self, -1, u'每个网络的最大引脚数：\ndefault=3  ', pos=(-35, 110), size=(200, -1), style=wx.ALIGN_RIGHT)
        # self.tip = wx.StaticText(self, -1, u'', pos=(145, 110), size=(150, -1), style=wx.ST_NO_AUTORESIZE)

        self.tc1 = wx.TextCtrl(self, -1, '', pos=(100, 30), size=(70, -1), name='TC01', style=wx.TE_CENTER)
        self.tc2 = wx.TextCtrl(self, -1, '', pos=(270, 30), size=(70, -1), name='TC02', style=wx.TE_CENTER)
        self.tc3 = wx.TextCtrl(self, -1, '', pos=(100, 70), size=(70, -1), name='TC03', style=wx.TE_CENTER)
        self.tc4 = wx.TextCtrl(self, -1, '', pos=(270, 70), size=(70, -1), name='TC04', style=wx.TE_CENTER)
        self.tc5 = wx.TextCtrl(self, -1, '', pos=(170, 110), size=(70, -1), name='TC05', style=wx.TE_CENTER)

        self.button1 = wx.Button(self, -1, u'生成布线问题', pos=(60, 160))
        self.button1.Bind(wx.EVT_BUTTON, self.OnButtonClick1)
        self.button2 = wx.Button(self, -1, u'开始布线', pos=(200, 160))
        self.button2.Bind(wx.EVT_BUTTON, self.OnButtonClick2)

    def OnButtonClick1(self, event):
        filename = 'random_benchmark.gr'
        grid_x = 32
        grid_y = 32
        grid_z = 2
        net_num = 3
        max_pin_num = 3
        if self.tc1.GetValue().isdigit():
            grid_x = int(self.tc1.GetValue())
        if self.tc2.GetValue().isdigit():
            grid_y = int(self.tc2.GetValue())
        if self.tc3.GetValue().isdigit():
            grid_z = int(self.tc3.GetValue())
        if self.tc4.GetValue().isdigit():
            net_num = int(self.tc4.GetValue())
        if self.tc5.GetValue().isdigit():
            max_pin_num = int(self.tc5.GetValue())
        generator(filename, [grid_x, grid_y], grid_z, net_num, max_pin_num)
        gp = grid_parameters(filename)
        env = GridEnv(gp)
        draw_origin_grid_plot(env)

    def OnButtonClick2(self, event):
        filename = 'random_benchmark.gr'
        if os.path.isfile(filename):
            gp = grid_parameters(filename)
            gridEnv = GridEnv(gp)
            while gridEnv.episode < 5:
                gridEnv.reset()
                if gridEnv.episode == 5:
                    draw_grid_plot(gridEnv)
                if gridEnv.episode == 5:
                    break
                gridEnv.breakup()
                route, cost = AStarSolver.a_star_route(gridEnv.init_pos, gridEnv.goal_pos,
                                                       gridEnv.occupied_coord, gridEnv.netPinSet, gp['gridSize'])
                # gridEnv.route = route
                # gridEnv.set_route(route)
                # gridEnv.cost = cost
                gridEnv.update(route, cost)
        else:
            er_msg = wx.MessageDialog(None, "布线问题不存在", "错误信息提示", wx.YES_DEFAULT | wx.ICON_ERROR)
            if er_msg.ShowModal() == wx.ID_YES:  # 如果点击了提示框的确定按钮
                er_msg.Destroy()  # 则关闭提示框


class MainFrame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, u'基于A*算法实现的多端网络布线器', size=(370, 280),
                          style=wx.MINIMIZE_BOX | wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX)
        self.SetBackgroundColour(wx.Colour(240, 240, 240))
        self.Center()

        icon = wx.Icon(APP_ICON, wx.BITMAP_TYPE_ICO)
        self.SetIcon(icon)

        self.title_text = wx.StaticText(self, -1, u'多端网络布线器', pos=(100, 80), style=wx.ALIGN_CENTER)
        # self.title_text.SetBackgroundColour('white')  # 设置背景颜色
        self.title_text.SetForegroundColour('black')  # 设置文本颜色
        font = wx.Font(14, wx.ROMAN, wx.ITALIC, wx.NORMAL)
        self.title_text.SetFont(font)

        self.button1 = wx.Button(self, -1, u'随机生成布线问题', pos=(40, 140))
        self.button1.Bind(wx.EVT_BUTTON, self.OnButtonClick1)

        self.button2 = wx.Button(self, -1, u'自定义生成布线问题', pos=(200, 140))
        self.button2.Bind(wx.EVT_BUTTON, self.OnButtonClick2)

    def OnButtonClick1(self, event):
        random_frame = RandomFrame()
        random_frame.Show()

    def OnButtonClick2(self, event):
        custom_frame = CustomizeFrame()
        custom_frame.Show()


if __name__ == "__main__":
    app = wx.App()
    frame = MainFrame()
    frame.Show()
    app.MainLoop()
