del /s /f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl %1.pdf

pdflatex refman -job-name=%1
echo ----
makeindex refman.idx
echo ----
pdflatex refman -job-name=%1

setlocal enabledelayedexpansion
set count=8
:repeat
set content=X
for /F "tokens=*" %%T in ( 'findstr /C:"Rerun LaTeX" refman.log' ) do set content="%%~T"
if !content! == X for /F "tokens=*" %%T in ( 'findstr /C:"Rerun to get cross-references right" refman.log' ) do set content="%%~T"
if !content! == X goto :skip
set /a count-=1
if !count! EQU 0 goto :skip

echo ----
pdflatex refman -job-name=%1
goto :repeat
:skip
endlocal
makeindex refman.idx
pdflatex refman -job-name=%1
