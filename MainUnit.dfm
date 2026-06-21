object MainForm: TMainForm
  Left = 0
  Top = 0
  Caption = 'GPS Receiver'
  ClientHeight = 703
  ClientWidth = 975
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -12
  Font.Name = 'Segoe UI'
  Font.Style = []
  Menu = MainMenu1
  TextHeight = 15
  object Splitter1: TSplitter
    Left = 0
    Top = 433
    Width = 975
    Height = 5
    Cursor = crVSplit
    Align = alTop
    MinSize = 120
  end
  object memoResults: TMemo
    Left = 0
    Top = 438
    Width = 975
    Height = 265
    Align = alClient
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -12
    Font.Name = 'Consolas'
    Font.Style = []
    ParentFont = False
    ScrollBars = ssBoth
    TabOrder = 0
  end
  object Panel1: TPanel
    Left = 0
    Top = 0
    Width = 975
    Height = 41
    Align = alTop
    TabOrder = 1
    object lblFile: TLabel
      Left = 4
      Top = 4
      Width = 102
      Height = 29
      Alignment = taCenter
      AutoSize = False
      Caption = 'Opened File'
      Layout = tlCenter
    end
    object editFile: TEdit
      Left = 112
      Top = 2
      Width = 273
      Height = 23
      TabOrder = 0
    end
  end
  object PageControl1: TPageControl
    Left = 0
    Top = 41
    Width = 975
    Height = 392
    ActivePage = tsAcquisition
    Align = alTop
    TabOrder = 2
    object tsAcquisition: TTabSheet
      Caption = 'Aquisition'
      DesignSize = (
        967
        362)
      object btnAcquire: TButton
        Left = 8
        Top = 8
        Width = 105
        Height = 25
        Caption = 'Acquire'
        TabOrder = 0
        OnClick = btnAcquireClick
      end
      object Chart1: TChart
        Left = 3
        Top = 40
        Width = 961
        Height = 318
        Legend.Alignment = laTop
        Legend.TopPos = 0
        Title.Text.Strings = (
          'TChart')
        Title.Visible = False
        View3D = False
        TabOrder = 1
        Anchors = [akLeft, akTop, akRight, akBottom]
        DefaultCanvas = 'TGDIPlusCanvas'
        ColorPaletteIndex = 13
        object Series1: TBarSeries
          Marks.OnTop = True
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Bar'
          YValues.Order = loNone
        end
      end
    end
    object tsTracking: TTabSheet
      Caption = 'Tracking'
      ImageIndex = 1
      DesignSize = (
        967
        362)
      object btnTrack: TButton
        Left = 8
        Top = 8
        Width = 120
        Height = 25
        Caption = 'Track Acquired'
        TabOrder = 0
        OnClick = btnTrackClick
      end
      object sgChannels: TStringGrid
        Left = 8
        Top = 40
        Width = 336
        Height = 312
        Anchors = [akLeft, akTop, akBottom]
        ColCount = 4
        DefaultColWidth = 80
        DefaultRowHeight = 22
        FixedCols = 0
        RowCount = 2
        Options = [goFixedVertLine, goFixedHorzLine, goVertLine, goHorzLine, goRowSelect]
        TabOrder = 1
        OnSelectCell = sgChannelsSelectCell
        ColWidths = (
          54
          92
          78
          92)
      end
    end
    object tsPosition: TTabSheet
      Caption = 'Position'
      ImageIndex = 2
      DesignSize = (
        967
        362)
      object btnFix: TButton
        Left = 8
        Top = 8
        Width = 120
        Height = 25
        Caption = 'Compute Fix'
        TabOrder = 0
        OnClick = btnFixClick
      end
      object sgSats: TStringGrid
        Left = 8
        Top = 40
        Width = 462
        Height = 312
        Anchors = [akLeft, akTop, akBottom]
        DefaultColWidth = 90
        DefaultRowHeight = 22
        FixedCols = 0
        RowCount = 2
        Options = [goFixedVertLine, goFixedHorzLine, goVertLine, goHorzLine, goRowSelect]
        TabOrder = 1
        ColWidths = (
          50
          104
          98
          98
          98)
      end
      object memoFix: TMemo
        Left = 476
        Top = 40
        Width = 491
        Height = 312
        Anchors = [akLeft, akTop, akRight, akBottom]
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -12
        Font.Name = 'Consolas'
        Font.Style = []
        ParentFont = False
        ReadOnly = True
        ScrollBars = ssBoth
        TabOrder = 2
      end
    end
  end
  object MainMenu1: TMainMenu
    Left = 512
    Top = 8
    object File1: TMenuItem
      Caption = 'File'
      object File2: TMenuItem
        Caption = 'Open'
        OnClick = File2Click
      end
      object Close1: TMenuItem
        Caption = 'Close'
      end
      object Close2: TMenuItem
        Caption = '-'
      end
      object Exit1: TMenuItem
        Caption = 'Exit'
      end
      object Exit2: TMenuItem
        Caption = 'Exit'
      end
    end
  end
  object OpenDialog1: TOpenDialog
    Left = 424
    Top = 8
  end
end
