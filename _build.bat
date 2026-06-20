@echo off
call "C:\Program Files (x86)\Embarcadero\Studio\37.0\bin\rsvars.bat" >nul
msbuild "C:\Users\andre\OneDrive\Documents\CCode\2026\GPSRx\GPSRx.cbproj" /t:Build /p:Config=%1 /p:Platform=Win64x /nologo /v:m
