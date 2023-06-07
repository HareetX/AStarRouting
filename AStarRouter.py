import wx

from DialogGUI import MainFrame

if __name__ == '__main__':
    app = wx.App()
    frame = MainFrame()
    frame.Show()
    app.MainLoop()
