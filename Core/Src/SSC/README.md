
after modify xlsx file, run below powershell script to regenerate SSC files

```powershell
cd Core/Src/SSC # folder contains SSC project files
& 'SSC OD Tool.exe' ./SSC-Device.xlsx /src ./src # update *.xml file
& 'SSC Tool Cmd.exe' -o . -p ./SSC-Device.esp -a ./SSC-Device.xml # update *.esp file
```
