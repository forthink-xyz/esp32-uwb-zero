@echo off
rem 执行 Doxygen 命令来解析配置文件

rem 设置 Doxygen 配置文件路径
set CONFIG_FILE=Doxyfile

rem 检查配置文件是否存在
if not exist %CONFIG_FILE% (
    echo Error: Doxygen configuration file "%CONFIG_FILE%" not found.
    exit /b 1
)

rem 调用 Doxygen 命令并解析配置文件
doxygen %CONFIG_FILE%

rem 检查是否生成了输出
if errorlevel 1 (
    echo Error: Doxygen encountered an error while generating documentation.
    exit /b 1
)

rem 调用 HTML Help Workshop 编译生成 index.chm
echo Compiling HTML help files...
hhc.exe docs\html\index.hhp

rem 检查是否生成了 index.chm
if not exist docs\html\index.chm (
    echo Error: HTML help compilation failed.
    exit /b 1
)

echo Documentation generated successfully.

rem 拷贝生成的 index.chm 文件到上层目录并改名为 UserGuide.chm
copy docs\html\index.chm ..\..\src\UserGuide.chm

pause
exit /b 0

