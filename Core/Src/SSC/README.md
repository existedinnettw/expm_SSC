
after modify xlsx file, run below powershell script to regenerate SSC files

```powershell
cd Core/Src/SSC # folder contains SSC project files
# Remove-Item -Recurse src -ErrorAction SilentlyContinue; Remove-Item *.xml -ErrorAction SilentlyContinue; Remove-Item *.bak -ErrorAction SilentlyContinue
# & 'SSC OD Tool.exe' ./SSC-Device.xlsx /src ./src # no longer needed
& 'SSC Tool Cmd.exe' -o . -p ./SSC-Device.esp -a ./SSC-Device.xlsx
git apply SSC-Device.patch
```
