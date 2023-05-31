import wx

APP_TITLE = u'动态布局'
APP_ICON = 'res/python.ico'


class Mainframe(wx.Frame):
    '''程序主窗口类，继承自wx.Frame'''

    def __init__(self, parent):
        '''构造函数'''

        wx.Frame.__init__(self, parent, -1, APP_TITLE)
        self.SetBackgroundColour(wx.Colour(240, 240, 240))
        self.SetSize((800, 600))
        self.Center()

        icon = wx.Icon(APP_ICON, wx.BITMAP_TYPE_ICO)
        self.SetIcon(icon)

        preview = wx.Panel(self, -1, style=wx.SUNKEN_BORDER)
        preview.SetBackgroundColour(wx.Colour(0, 0, 0))
        btn_random = wx.Button(self, -1, u'随机生成', size=(100, -1))
        tc_file = wx.TextCtrl(self, -1, '', style=wx.TE_MULTILINE)
        btn_customize = wx.Button(self, -1, u'指定生成', size=(100, -1))
        tc_result = wx.TextCtrl(self, -1, '', style=wx.TE_MULTILINE)

        sizer_customize = wx.StaticBoxSizer(wx.StaticBox(self, -1, u'自定义'), wx.VERTICAL)
        sizer_customize.Add(tc_file, 0, wx.ALIGN_CENTER | wx.ALL, 0)
        sizer_customize.Add(btn_customize, 0, wx.ALIGN_CENTER | wx.ALL, 0)

        sizer_right = wx.BoxSizer(wx.VERTICAL)
        sizer_right.Add(btn_random, 0, wx.ALL, 20)
        sizer_right.Add(sizer_customize, 0, wx.ALIGN_CENTER | wx.ALL, 0)
        sizer_right.Add(tc_result, 1, wx.ALL, 10)

        sizer_max = wx.BoxSizer()
        sizer_max.Add(preview, 1, wx.EXPAND | wx.LEFT | wx.TOP | wx.BOTTOM, 5)
        sizer_max.Add(sizer_right, 0, wx.EXPAND | wx.ALL, 0)

        self.SetAutoLayout(True)
        self.SetSizer(sizer_max)
        self.Layout()


class Mainapp(wx.App):
    def OnInit(self):
        self.SetAppName(APP_TITLE)
        self.Frame = Mainframe(None)
        self.Frame.Show()
        return True


if __name__ == "__main__":
    app = Mainapp()
    app.MainLoop()
