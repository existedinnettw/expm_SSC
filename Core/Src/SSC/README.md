
after modify xlsx file, run below powershell script to regenerate SSC files

```powershell
cd Core/Src/SSC # folder contains SSC project files
# Remove-Item -Recurse src -ErrorAction SilentlyContinue; Remove-Item *.xml -ErrorAction SilentlyContinue; Remove-Item *.bak -ErrorAction SilentlyContinue
# & 'SSC OD Tool.exe' ./SSC-Device.xlsx /src ./src # no longer needed
& 'SSC Tool Cmd.exe' -o . -p ./SSC-Device.esp -a ./SSC-Device.xlsx
cp src/SSC-Device.c src/SSC-Device.c.ori
git apply SSC-Device.patch
# after editing `src/SSC-Device.c`, regenerate patch
git diff --no-index src/SSC-Device.c.ori src/SSC-Device.c > SSC-Device.patch
```
